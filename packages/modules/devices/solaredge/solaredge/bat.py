#!/usr/bin/env python3
import logging
from typing import Dict, Union, Optional

from pymodbus.constants import Endian

from dataclass_utils import dataclass_from_dict
from modules.common import modbus
from modules.common.abstract_device import AbstractBat
from modules.common.component_state import BatState
from modules.common.component_type import ComponentDescriptor
from modules.common.fault_state import ComponentInfo, FaultState
from modules.common.modbus import ModbusDataType
from modules.common.simcount import SimCounter
from modules.common.store import get_bat_value_store
from modules.devices.solaredge.solaredge.config import SolaredgeBatSetup
import pymodbus

log = logging.getLogger(__name__)

FLOAT32_UNSUPPORTED = -0xffffff00000000000000000000000000


class SolaredgeBat(AbstractBat):
    # Define all possible registers with their data types
    REGISTERS = {
        "Battery1StateOfEnergy": (0xe184, ModbusDataType.FLOAT_32,),  # auch 0xf584
        "Battery1InstantaneousPower": (0xe174, ModbusDataType.FLOAT_32,),  # auch 0xf574
        "Battery2StateOfEnergy": (0xe284, ModbusDataType.FLOAT_32,),
        "Battery2InstantaneousPower": (0xe274, ModbusDataType.FLOAT_32,),
        "StorageControlMode": (0xe004, ModbusDataType.UINT_16,),
        "StorageChargeDischargeDefaultMode": (0xe00a, ModbusDataType.UINT_16,),
        "RemoteControlCommandMode": (0xe00d, ModbusDataType.UINT_16,),
        "RemoteControlCommandDischargeLimit": (0xe010, ModbusDataType.FLOAT_32,),
    }

    def __init__(self,
                 device_id: int,
                 component_config: Union[Dict, SolaredgeBatSetup],
                 tcp_client: modbus.ModbusTcpClient_) -> None:
        self.__device_id = device_id
        self.component_config = dataclass_from_dict(SolaredgeBatSetup, component_config)
        self.__tcp_client = tcp_client
        self.sim_counter = SimCounter(self.__device_id, self.component_config.id, prefix="speicher")
        self.store = get_bat_value_store(self.component_config.id)
        self.fault_state = FaultState(ComponentInfo.from_component_config(self.component_config))
        self.bat_minsoc = 10  # Mindest-SoC zur Steuerung des Speichers, ggf. self.component_config.configuration.bat_minsoc oder so
        self.last_mode = 'Unknown'

    def update(self) -> None:
        self.store.set(self.read_state())

    def read_state(self) -> BatState:
        unit = self.component_config.configuration.modbus_id

        """"
        if battery_index == 1:
            registers_to_read = [
                "Battery1InstantaneousPower",
                "Battery1StateOfEnergy",
            ]
            values = self._read_registers(registers_to_read, unit)
            power = values["Battery1InstantaneousPower"]
            soc = values["Battery1StateOfEnergy"]

        elif battery_index == 2:
            registers_to_read = [
                "Battery2InstantaneousPower",
                "Battery2StateOfEnergy",
            ]
            values = self._read_registers(registers_to_read, unit)
            power = values["Battery1InstantaneousPower"]
            soc = values["Battery1StateOfEnergy"]

        else:
            raise ValueError(f"Invalid battery_index: {battery_index}. Must be 1 or 2.")"
            """

        registers_to_read = [
            "Battery1InstantaneousPower",
            "Battery1StateOfEnergy",
        ]
        values = self._read_registers(registers_to_read, unit)
        power = values["Battery1InstantaneousPower"]
        soc = values["Battery1StateOfEnergy"]

        if power == FLOAT32_UNSUPPORTED:
            power = 0

        imported, exported = self.sim_counter.sim_count(power)

        bat_state = BatState(
            power=power,
            soc=soc,
            imported=imported,
            exported=exported
        )
        log.debug(f"Bat {self.__tcp_client.address}: {bat_state}")
        return bat_state

    def set_power_limit(self, power_limit: Optional[int]) -> None:
        unit = self.component_config.configuration.modbus_id
        power_limit_mode = power_limit_mode  # Einlesen Variable aus MQTT Topic fehlt noch...

        if power_limit_mode == 'no_limit':
            # Keine Steuerung nötig, Modus Immer gewählt.
            return

        registers_to_read = [
            "Battery1StateOfEnergy",
            "StorageControlMode",
            "RemoteControlCommandMode",
            "RemoteControlCommandDischargeLimit",
        ]
        values = self._read_registers(registers_to_read, unit)

        if values["Battery1StateOfEnergy"] <= self.bat_minsoc:
            # Keine Steuerunng, da Mindest-SoC unterschritten, ggf. externe Steuerung deaktivieren.
            if values["StorageControlMode"] != 2:
                log.debug(f"Mindest-SoC unterschritten, deaktiviere externe Steuerung.")
                values_to_write = {
                    "RemoteControlCommandDischargeLimit": 5000,
                    "StorageControlMode": 2,
                }
                self._write_registers(values_to_write, unit)
            return

        if power_limit is None and values["StorageControlMode"] == 2:
            # Kein Powerlimit gefordert, externe Steuerung bereits inaktiv.
            pass

        elif power_limit is None and values["StorageControlMode"] != 2:
            # Kein Powerlimit gefordert, externe Steuerung aktiv, externe Steuerung deaktivieren.
            log.debug(f"Keine Batteriesteuerung mehr gefordert, deaktiviere externe Steuerung.")
            values_to_write = {
                "RemoteControlCommandDischargeLimit": 5000,
                "StorageControlMode": 2,
            }
            self._write_registers(values_to_write, unit)

        elif power_limit == 0 and values["StorageControlMode"] != 4 or values["RemoteControlCommandMode"] != 1:
            # externe Steuerung aktivieren, Speichermodus "Mit PV-Überschuss laden", Speicher wird nicht entladen.
            values_to_write = {
                "StorageControlMode": 4,
                "StorageChargeDischargeDefaultMode": 1,
                "RemoteControlCommandMode": 1,
            }
            self._write_registers(values_to_write, unit)
            log.debug("Batteriesteuerung aktiviert. Modus 'Mit PV-Überschuss laden'.")

        elif power_limit > 0:
            # Speichersteuerung Hausverbrauch
            if values["StorageControlMode"] != 4 or values["RemoteControlCommandMode"] != 7:
                # externe Steuerung aktivieren, Speichermodus "Maximaler Eigenverbrauch".
                values_to_write = {
                    "StorageControlMode": 4,
                    "StorageChargeDischargeDefaultMode": 7,
                    "RemoteControlCommandMode": 7,
                }
                self._write_registers(values_to_write, unit)
                log.debug(f"Batteriesteuerung aktiviert. Modus maximaler Eigenverbrauch")

            if values["RemoteControlCommandDischargeLimit"] not in range (power_limit-10, power_limit+10):
                # Entladelimit setzen, nur bei Abweichung von mehr als 10W, max. 5000W.
                values_to_write = {
                    "RemoteControlCommandDischargeLimit": int(min(power_limit, 5000))
                }
                self._write_registers(values_to_write, unit)
                log.debug(f"Powerlimit gesetzt {power_limit} W")

    def _read_registers(self, register_names: list, unit: int) -> Dict[str, Union[int, float]]:
        values = {}
        for key in register_names:
            log.debug(f"Bat raw values {self.__tcp_client.address}: {values}")
            address, data_type = self.REGISTERS[key]
            values[key] = self.__tcp_client.read_holding_registers(
            address, data_type, wordorder=Endian.Little, unit=unit
            )
        log.debug(f"Bat raw values {self.__tcp_client.address}: {values}")
        return values

    def _write_registers(self, values_to_write: Dict[str, Union[int, float]], unit: int) -> None:
        for key, value in values_to_write.items():
            address, data_type = self.REGISTERS[key]
            encoded_value = self._encode_value(value, data_type)
            self.__tcp_client.write_registers(address, encoded_value, unit=unit)
            log.debug(f"Neuer Wert {encoded_value} in Register {address} geschrieben.")

    def _encode_value(self, value: Union[int, float], data_type: ModbusDataType) -> list:
        builder = pymodbus.payload.BinaryPayloadBuilder(
                byteorder=pymodbus.constants.Endian.Big,
                wordorder=pymodbus.constants.Endian.Little
            )
        encode_methods = {
            ModbusDataType.UINT_32: builder.add_32bit_uint,
            ModbusDataType.INT_32: builder.add_32bit_int,
            ModbusDataType.UINT_16: builder.add_16bit_uint,
            ModbusDataType.INT_16: builder.add_16bit_int,
            ModbusDataType.FLOAT_32: builder.add_32bit_float,
        }

        if data_type in encode_methods:
            encode_methods[data_type](int(value))
        else:
            raise ValueError(f"Unsupported data type: {data_type}")

        return builder.to_registers()


component_descriptor = ComponentDescriptor(configuration_factory=SolaredgeBatSetup)
