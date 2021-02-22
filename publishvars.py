import re
import sys
import time
import fileinput
import argparse
import os
import paho.mqtt.client as mqtt
from subprocess import Popen
import json

class pubvars():
    def __init__(self):
        self.ramdiskpath="/var/www/html/openWB/ramdisk/"

        self.parser = argparse.ArgumentParser(description='openWB MQTT Publisher')
        self.parser.add_argument('--qos', '-q', metavar='qos', type=int, help='The QOS setting', default=0)
        self.parser.add_argument('--retain', '-r', dest='retain', action='store_true', help='If true, retain this publish')
        self.parser.set_defaults(retain=False)

        self.args = self.parser.parse_args()

        self.client = mqtt.Client("openWB-python-bulkpublisher-" + str(os.getpid()))
        self.client.connect("localhost")

        self.vars()

        self.client.loop(timeout=2.0)
        self.client.disconnect()

    def pub_float(self, var, topic):
        """
        """
        f = open( self.ramdiskpath+var , 'r')
        value =f.read()
        f.close()
        if value == '\n':
            value=float(0)
        else:
            value=float(value)
        self.client.publish(topic, payload=json.dumps(value), qos=self.args.qos, retain=self.args.retain)

    def pub_int(self, var, topic):
        """
        """
        f = open( self.ramdiskpath+var , 'r')
        value =int(f.read().rstrip('\n'))
        f.close()
        self.client.publish(topic, payload=json.dumps(value), qos=self.args.qos, retain=self.args.retain)

    def pub_str(self, var, topic):
        """
        """
        f = open( self.ramdiskpath+var , 'r')
        value =str(f.read()).rstrip('\n')
        f.close()
        self.client.publish(topic, payload=json.dumps(value), qos=self.args.qos, retain=self.args.retain)

    def pub_float_list(self, var, topic):
        """
        """
        value=[]
        for item in var:
            f = open( self.ramdiskpath+item , 'r')
            value =f.read()
            f.close()
            if value == '\n':
                value=float(0)
            else:
                value=float(value)
        self.client.publish(topic, payload=json.dumps(value), qos=self.args.qos, retain=self.args.retain)

    def vars(self):
        """
        """
        self.pub_float("llv1", "openWB/chargepoint/1/voltage")

        #counter
        #evu
        self.pub_float_list(["bezuga1", "bezuga2", "bezuga3"], "openWB/counter/evu/get/current")
        self.pub_float_list(["bezugw1", "bezugw2", "bezugw3"], "openWB/counter/evu/get/power_phase")
        self.pub_float_list(["evuv1", "evuv2", "evuv3"], "openWB/counter/evu/get/voltage")
        self.pub_float("evuhz", "openWB/counter/evu/get/frequency")
        self.pub_float_list(["evupf1", "evupf2", "evupf3"], "openWB/counter/evu/get/power_factor")
        self.pub_float("einspeisungkwh", "openWB/counter/evu/get/exported_wh")
        self.pub_float("bezugkwh", "openWB/counter/evu/get/imported_wh")
        # self.pub_float("evu-live.graph", "openWB/")
        # self.pub_float("evu.graph", "openWB/")
        self.pub_float("wattbezug", "openWB/counter/evu/get/power")
        #chargepoint
        # self.pub_float("evsedintestlp1", "openWB/")
        # self.pub_float("evsedintestlp2", "openWB/")
        # self.pub_float("evsedintestlp3", "openWB/")
        # self.pub_float("restzeitlp1m", "openWB/")
        # self.pub_float("restzeitlp2m", "openWB/")
        # self.pub_float("restzeitlp3m", "openWB/")
        self.pub_int("aktgeladen",    "openWB/chargepoint/1/get/daily_charged_kwh")
        self.pub_int("aktgeladens1",  "openWB/chargepoint/2/get/daily_charged_kwh")
        self.pub_int("aktgeladens2",  "openWB/chargepoint/3/get/daily_charged_kwh")
        self.pub_int("aktgeladenlp4", "openWB/chargepoint/4/get/daily_charged_kwh")
        self.pub_int("aktgeladenlp5", "openWB/chargepoint/5/get/daily_charged_kwh")
        self.pub_int("aktgeladenlp6", "openWB/chargepoint/6/get/daily_charged_kwh")
        self.pub_int("aktgeladenlp7", "openWB/chargepoint/7/get/daily_charged_kwh")
        self.pub_int("aktgeladenlp8", "openWB/chargepoint/8/get/daily_charged_kwh")
        # self.pub_int("chargestat",    "openWB/chargepoint/1/get/charge_state")
        # self.pub_int("chargestats1",  "openWB/chargepoint/2/get/charge_state")
        # self.pub_int("chargestatlp3", "openWB/chargepoint/3/get/charge_state")
        # self.pub_int("chargestatlp4", "openWB/chargepoint/4/get/charge_state")
        # self.pub_int("chargestatlp5", "openWB/chargepoint/5/get/charge_state")
        # self.pub_int("chargestatlp6", "openWB/chargepoint/6/get/charge_state")
        # self.pub_int("chargestatlp7", "openWB/chargepoint/7/get/charge_state")
        # self.pub_int("chargestatlp8", "openWB/chargepoint/8/get/charge_state")
        self.pub_int("ladestatus",    "openWB/chragepoint/1/get/charge_state")
        self.pub_int("ladestatuss1",  "openWB/chragepoint/2/get/charge_state")
        self.pub_int("ladestatuss2",  "openWB/chragepoint/3/get/charge_state")
        self.pub_int("ladestatuslp4", "openWB/chragepoint/4/get/charge_state")
        self.pub_int("ladestatuslp5", "openWB/chragepoint/5/get/charge_state")
        self.pub_int("ladestatuslp6", "openWB/chragepoint/6/get/charge_state")
        self.pub_int("ladestatuslp7", "openWB/chragepoint/7/get/charge_state")
        self.pub_int("ladestatuslp8", "openWB/chragepoint/8/get/charge_state")
        # self.pub_float("gelrlp1", "openWB/")
        # self.pub_float("gelrlp2", "openWB/")
        # self.pub_float("gelrlp3", "openWB/")
        # self.pub_float("ladeleistunglp4", "openWB/")
        # self.pub_float("ladeleistunglp5", "openWB/")
        # self.pub_float("ladeleistunglp6", "openWB/")
        # self.pub_float("ladeleistunglp7", "openWB/")
        # self.pub_float("ladeleistunglp8", "openWB/")
        # self.pub_float("ladungaktivlp1", "openWB/")
        # self.pub_float("ladungaktivlp2", "openWB/")
        # self.pub_float("ladungaktivlp3", "openWB/")
        self.pub_float_list(["lla1"   , "lla2"  , "lla3"   ], "openWB/chargepoint/1/get/current")
        self.pub_float_list(["llas11" , "llas12", "llas13" ] , "openWB/chargepoint/2/get/current")
        self.pub_float_list(["llas21" , "llas22", "llas23" ] , "openWB/chargepoint/3/get/current")
        self.pub_float_list(["lla1lp4", "lla2lp4", "lla3lp4"], "openWB/chargepoint/4/get/current")
        self.pub_float_list(["lla1lp5", "lla2lp5", "lla3lp5"], "openWB/chargepoint/5/get/current")
        self.pub_float_list(["lla1lp6", "lla2lp6", "lla3lp6"], "openWB/chargepoint/6/get/current")
        self.pub_float_list(["lla1lp7", "lla2lp7", "lla3lp7"], "openWB/chargepoint/7/get/current")
        self.pub_float_list(["lla1lp8", "lla2lp8", "lla3lp8"], "openWB/chargepoint/8/get/current")
        self.pub_float("llkwh",    "openWB/chragepoint/1/get/counter_kwh")
        self.pub_float("llkwhs1",  "openWB/chragepoint/2/get/counter_kwh")
        self.pub_float("llkwhs2",  "openWB/chragepoint/3/get/counter_kwh")
        self.pub_float("llkwhlp4", "openWB/chragepoint/4/get/counter_kwh")
        self.pub_float("llkwhlp5", "openWB/chragepoint/5/get/counter_kwh")
        self.pub_float("llkwhlp6", "openWB/chragepoint/6/get/counter_kwh")
        self.pub_float("llkwhlp7", "openWB/chragepoint/7/get/counter_kwh")
        self.pub_float("llkwhlp8", "openWB/chragepoint/8/get/counter_kwh")
        self.pub_float("llsoll",    "openWB/chragepoint/1/get/energy_to_charge")
        self.pub_float("llsolls1",  "openWB/chragepoint/2/get/energy_to_charge")
        self.pub_float("llsolls2",  "openWB/chragepoint/3/get/energy_to_charge")
        self.pub_float("llsolllp4", "openWB/chragepoint/4/get/energy_to_charge")
        self.pub_float("llsolllp5", "openWB/chragepoint/5/get/energy_to_charge")
        self.pub_float("llsolllp6", "openWB/chragepoint/6/get/energy_to_charge")
        self.pub_float("llsolllp7", "openWB/chragepoint/7/get/energy_to_charge")
        self.pub_float("llsolllp8", "openWB/chragepoint/8/get/energy_to_charge")
        self.pub_float_list(["llv1"   , "llv2"   , "llv3"   ], "openWB/chragepoint/1/get/voltage")
        self.pub_float_list(["llvs11" , "llvs12" , "llvs13" ], "openWB/chragepoint/2/get/voltage")
        self.pub_float_list(["llvs21" , "llvs22" , "llvs23" ], "openWB/chragepoint/3/get/voltage")
        self.pub_float_list(["llv1lp4", "llv2lp4", "llv3lp4"], "openWB/chragepoint/4/get/voltage")
        self.pub_float_list(["llv1lp5", "llv2lp5", "llv3lp5"], "openWB/chragepoint/5/get/voltage")
        self.pub_float_list(["llv1lp6", "llv2lp6", "llv3lp6"], "openWB/chragepoint/6/get/voltage")
        self.pub_float_list(["llv1lp7", "llv2lp7", "llv3lp7"], "openWB/chragepoint/7/get/voltage")
        self.pub_float_list(["llv1lp8", "llv2lp8", "llv3lp8"], "openWB/chragepoint/8/get/voltage")
        self.pub_int("pluggedladungbishergeladen",    "openWB/chargepoint/1/get/charged_since_plugged_kwh")
        self.pub_int("pluggedladungbishergeladenlp2", "openWB/chargepoint/2/get/charged_since_plugged_kwh")
        self.pub_int("pluggedladungbishergeladenlp3", "openWB/chargepoint/3/get/charged_since_plugged_kwh")
        self.pub_int("pluggedladungbishergeladenlp4", "openWB/chargepoint/4/get/charged_since_plugged_kwh")
        self.pub_int("pluggedladungbishergeladenlp5", "openWB/chargepoint/5/get/charged_since_plugged_kwh")
        self.pub_int("pluggedladungbishergeladenlp6", "openWB/chargepoint/6/get/charged_since_plugged_kwh")
        self.pub_int("pluggedladungbishergeladenlp7", "openWB/chargepoint/7/get/charged_since_plugged_kwh")
        self.pub_int("pluggedladungbishergeladenlp8", "openWB/chargepoint/8/get/charged_since_plugged_kwh")
        self.pub_int("plugstat",    "openWB/chargepoint/1/get/plug_state")
        self.pub_int("plugstats1",  "openWB/chargepoint/2/get/plug_state")
        self.pub_int("plugstatlp3", "openWB/chargepoint/3/get/plug_state")
        self.pub_int("plugstatlp4", "openWB/chargepoint/4/get/plug_state")
        self.pub_int("plugstatlp5", "openWB/chargepoint/5/get/plug_state")
        self.pub_int("plugstatlp6", "openWB/chargepoint/6/get/plug_state")
        self.pub_int("plugstatlp7", "openWB/chargepoint/7/get/plug_state")
        self.pub_int("plugstatlp8", "openWB/chargepoint/8/get/plug_state")
        # self.pub_float("llaltnv", "openWB/")
        # self.pub_float("llhz", "openWB/")
        # self.pub_float("llkombiniert", "openWB/")
        self.pub_float("llkwhges", "openWB/chargepoint/get/counter_all_kwh")
        self.pub_float_list(["llpf1", "llpf2", "llpf3"], "openWB/chargepoint/1/get/power_factor")
        self.pub_float("llaktuell",    "openWB/chargepoint/1/get/power_w")
        self.pub_float("llaktuells1",  "openWB/chargepoint/2/get/power_w")
        self.pub_float("llaktuells2",  "openWB/chargepoint/3/get/power_w")
        self.pub_float("llaktuelllp4", "openWB/chargepoint/4/get/power_w")
        self.pub_float("llaktuelllp5", "openWB/chargepoint/5/get/power_w")
        self.pub_float("llaktuelllp6", "openWB/chargepoint/6/get/power_w")
        self.pub_float("llaktuelllp7", "openWB/chargepoint/7/get/power_w")
        self.pub_float("llaktuelllp8", "openWB/chargepoint/8/get/power_w")
        # self.pub_float("nachtladen2state", "openWB/")
        # self.pub_float("nachtladen2states1", "openWB/")
        # self.pub_float("nachtladenstate", "openWB/")
        # self.pub_float("nachtladenstates1", "openWB/")
        # self.pub_float("pluggedtimer1", "openWB/")
        # self.pub_float("pluggedtimer2", "openWB/")
        # self.pub_float("progevsedinlp1", "openWB/")
        # self.pub_float("progevsedinlp12000", "openWB/")
        # self.pub_float("progevsedinlp12007", "openWB/")
        # self.pub_float("progevsedinlp2", "openWB/")
        # self.pub_float("progevsedinlp22000", "openWB/")
        # self.pub_float("progevsedinlp22007", "openWB/")
        # self.pub_float("cpulp1counter", "openWB/")
        # self.pub_float("soc", "openWB/")
        # self.pub_float("soc-live.graph", "openWB/")
        # self.pub_float("soc.graph", "openWB/")
        # self.pub_float("soc1", "openWB/")
        # self.pub_float("soc1vorhanden", "openWB/")
        # self.pub_float("tmpsoc", "openWB/")
        # self.pub_float("tmpsoc1", "openWB/")
        # self.pub_float("zielladenkorrektura", "openWB/")
        # self.pub_float("soctimer", "openWB/")
        # self.pub_float("soctimer1", "openWB/")
        # self.pub_float("evsemodbustimer", "openWB/")
        # self.pub_float("llog1", "openWB/")
        # self.pub_float("llogs1", "openWB/")
        # self.pub_float("llogs2", "openWB/")
        #common files
        self.pub_float("pluggedladunglp1startkwh", "openWB/vehicle/1/get/counter_at_plugtime_kwh")
        self.pub_float("pluggedladunglp2startkwh", "openWB/vehicle/2/get/counter_at_plugtime_kwh")
        self.pub_float("pluggedladunglp3startkwh", "openWB/vehicle/3/get/counter_at_plugtime_kwh")
        self.pub_float("pluggedladunglp4startkwh", "openWB/vehicle/4/get/counter_at_plugtime_kwh")
        self.pub_float("pluggedladunglp5startkwh", "openWB/vehicle/5/get/counter_at_plugtime_kwh")
        self.pub_float("pluggedladunglp6startkwh", "openWB/vehicle/6/get/counter_at_plugtime_kwh")
        self.pub_float("pluggedladunglp7startkwh", "openWB/vehicle/7/get/counter_at_plugtime_kwh")
        self.pub_float("pluggedladunglp8startkwh", "openWB/vehicle/8/get/counter_at_plugtime_kwh")
        # self.pub_float("pluggedladungaktlp${i}:openWB/lp/${i}/pluggedladungakt:0" \", "openWB/")
        # self.pub_float("lp${i}phasen::0" \", "openWB/")
        # self.pub_float("lp${i}enabled::1" \", "openWB/")
        # self.pub_float("restzeitlp${i}::--" \", "openWB/")
        # self.pub_float("autolockstatuslp${i}::0" \", "openWB/")
        # self.pub_float("autolockconfiguredlp${i}::0" \", "openWB/")
        # self.pub_float("lp${i}sofortll::10" \", "openWB/")
        self.pub_float("rfidlp1", "openWB/chargepoint/1/config/rfid")
        self.pub_float("rfidlp2", "openWB/chargepoint/2/config/rfid")
        self.pub_float("rfidlp3", "openWB/chargepoint/3/config/rfid")
        self.pub_float("rfidlp4", "openWB/chargepoint/4/config/rfid")
        self.pub_float("rfidlp5", "openWB/chargepoint/5/config/rfid")
        self.pub_float("rfidlp6", "openWB/chargepoint/6/config/rfid")
        self.pub_float("rfidlp7", "openWB/chargepoint/7/config/rfid")
        self.pub_float("rfidlp8", "openWB/chargepoint/8/config/rfid")
        self.pub_float("boolstopchargeafterdisclp1", "openWB/vehicle/template/charge_template/1/disable_after_unplug")
        self.pub_float("boolstopchargeafterdisclp2", "openWB/vehicle/template/charge_template/2/disable_after_unplug")
        self.pub_float("boolstopchargeafterdisclp3", "openWB/vehicle/template/charge_template/3/disable_after_unplug")
        self.pub_float("boolstopchargeafterdisclp4", "openWB/vehicle/template/charge_template/4/disable_after_unplug")
        self.pub_float("boolstopchargeafterdisclp5", "openWB/vehicle/template/charge_template/5/disable_after_unplug")
        self.pub_float("boolstopchargeafterdisclp6", "openWB/vehicle/template/charge_template/6/disable_after_unplug")
        self.pub_float("boolstopchargeafterdisclp7", "openWB/vehicle/template/charge_template/7/disable_after_unplug")
        self.pub_float("boolstopchargeafterdisclp8", "openWB/vehicle/template/charge_template/8/disable_after_unplug")
        # self.pub_float("mqttzielladenaktivlp${i}::-1" \", "openWB/")
        # self.pub_float("mqttmsmoduslp${i}::-1" \", "openWB/")
        # self.pub_float("mqttlp${i}name::Lp${i}" \", "openWB/")
        # self.pub_float("mqttdisplaylp${i}max::-1" \", "openWB/")
        # self.pub_float("mqttautolockstatuslp${i}::-1" \", "openWB/")
        # self.pub_float("mqttautolockconfiguredlp${i}::-1"", "openWB/")
        #pv
        self.pub_float("daily_pvkwhk", "openWB/pv/get/daily_yield_kwh")
        self.pub_float("daily_pvkwhk1", "openWB/pv/modules/1/get/daily_yield_kwh")
        self.pub_float("daily_pvkwhk2", "openWB/pv/modules/2/get/daily_yield_kwh")
        self.pub_float("monthly_pvkwhk", "openWB/pv/get/monthly_yield_kwh")
        self.pub_float("monthly_pvkwhk1", "openWB/pv/modules/1/get/monthly_yield_kwh")
        self.pub_float("monthly_pvkwhk2", "openWB/pv/modules/2/get/monthly_yield_kwh")
        self.pub_int("nurpv70dynstatus", "openWB/pv/config/feed_in_yield")
        # self.pub_float("pv-live.graph", "openWB/")
        # self.pub_float("pv.graph", "openWB/")
        self.pub_float("pv1watt", "openWB/pv/modules/1/get/power")
        self.pub_float_list(["pv2a1", "pv2a2", "pv2a3"], "openWB/pv/modules/2/get/current")
        self.pub_float("pv2kwh", "openWB/pv/modules/2/get/counter_kwh")
        self.pub_float("pv2watt", "openWB/pv/modules/2/get/power")
        # self.pub_float("pvcounter", "openWB/")
        # self.pub_float("pvecounter", "openWB/")
        self.pub_float("pvkwh", "openWB/pv/get/counter_kwh")
        self.pub_float("pvkwhk", "openWB/pv/get/counter_kwh")
        self.pub_float("pvkwhk1", "openWB/pv/modules/1/get/counter_kwh")
        self.pub_float("pvkwhk2", "openWB/pv/modules/2/get/counter_kwh")
        self.pub_int("pv1vorhanden", "openWB/pv/modules/1")
        self.pub_int("pv2vorhanden", "openWB/pv/modules/2")
        self.pub_float("pvwatt", "openWB/pv/get/power")
        self.pub_float("pvwatt1", "openWB/pv/modules/1/get/power")
        self.pub_float("pvwatt2", "openWB/pv/modules/2/get/power")
        self.pub_float("yearly_pvkwhk", "openWB/pv/get/yearly_yield_kwh")
        self.pub_float("yearly_pvkwhk1", "openWB/pv/modules/1/get/yearly_yield_kwh")
        self.pub_float("yearly_pvkwhk2", "openWB/pv/modules/2/get/yearly_yield_kwh")
        #bat
        self.pub_int("speicher", "openWB/bat/modules/1")
        self.pub_float("speicherekwh", "openWB/bat/modules/1/get/exported_wh")
        self.pub_float("speicherikwh", "openWB/bat/modules/1/get/imported_wh")
        self.pub_float("speicherleistung", "openWB/bat/modules/1/get/power")
        #unused self.pub_float("speicherleistung1", "openWB/bat/modules/1/get/")
        #unused self.pub_float("speicherleistung2", "openWB/bat/modules/1/get/")
        self.pub_int("speichersoc", "openWB/bat/modules/1/get/soc")
        #unused self.pub_float("speichersoc2", "openWB/bat/modules/1/get/")
        #temp mqtt
        # self.pub_float("mqttdurchslp2", "openWB/")
        # self.pub_float("mqttdurchslp3", "openWB/")
        # self.pub_float("mqttgelrlp1", "openWB/")
        # self.pub_float("mqttgelrlp2", "openWB/")
        # self.pub_float("mqttgelrlp3", "openWB/")
        # self.pub_float("mqttladeleistunglp1", "openWB/")
        # self.pub_float("mqttladeleistungs1", "openWB/")
        # self.pub_float("mqttladeleistungs2", "openWB/")
        # self.pub_float("mqttlastchargestat", "openWB/")
        # self.pub_float("mqttlastchargestats1", "openWB/")
        # self.pub_float("mqttlastladestatus", "openWB/")
        # self.pub_float("mqttlastplugstat", "openWB/")
        # self.pub_float("mqttlastplugstats1", "openWB/")
        # self.pub_float("mqttpv1vorhanden", "openWB/")
        # self.pub_float("mqttpv2vorhanden", "openWB/")
        # self.pub_float("mqttetprovidermaxprice", "openWB/")
        # self.pub_float("mqttetproviderprice", "openWB/")
        # self.pub_float("mqttlademkwhs1", "openWB/")
        # self.pub_float("mqttlademkwhs2", "openWB/")
        # self.pub_float("mqttllsolls1", "openWB/")
        # self.pub_float("mqttllsolls2", "openWB/")
        # self.pub_float("mqttsoc1", "openWB/")
        # self.pub_float("mqttspeicherleistung", "openWB/")
        # self.pub_float("mqttspeichervorhanden", "openWB/")
        # self.pub_float("mqttlastmanagement", "openWB/")
        # self.pub_float("mqttlastmanagements1", "openWB/")
        # self.pub_float("mqttlastmanagements2", "openWB/")
        # self.pub_float("mqttlastmanagementlp4", "openWB/")
        # self.pub_float("mqttlastmanagementlp5", "openWB/")
        # self.pub_float("mqttlastmanagementlp6", "openWB/")
        # self.pub_float("mqttlastmanagementlp7", "openWB/")
        # self.pub_float("mqttlastmanagementlp8", "openWB/")
        # self.pub_float("mqttspeichersoc", "openWB/")
        # self.pub_float("mqttrfidlasttag", "openWB/")
        # self.pub_float("mqttrfidlp1", "openWB/")
        # self.pub_float("mqttrfidlp2", "openWB/")
        #rfid
        # self.pub_float("rfidlist", "openWB/")
        # self.pub_float("rfidlasttag", "openWB/")
        # self.pub_float("rfidlp1", "openWB/")
        # self.pub_float("rfidlp2", "openWB/")
        # self.pub_float("readtag", "openWB/")
        #diverse Dateien
        #slave self.pub_float("AllowedTotalCurrentPerPhase", "openWB/")
        #slave self.pub_float("ChargingVehiclesOnL1", "openWB/")
        #slave self.pub_float("ChargingVehiclesOnL2", "openWB/")
        #slave self.pub_float("ChargingVehiclesOnL3", "openWB/")
        #slave self.pub_float("TotalCurrentConsumptionOnL1", "openWB/")
        #slave self.pub_float("TotalCurrentConsumptionOnL2", "openWB/")
        #slave self.pub_float("TotalCurrentConsumptionOnL3", "openWB/")
        # self.pub_float("autolocktimer", "openWB/")
        # self.pub_float("blockall", "openWB/")
        # self.pub_float("date-live.graph", "openWB/")
        # self.pub_float("date.graph", "openWB/")
        # self.pub_float("devicetotal_watt", "openWB/")
        # self.pub_float("etprovidermaxprice", "openWB/")
        # self.pub_float("etproviderprice", "openWB/")
        # self.pub_float("ev-live.graph", "openWB/")
        # self.pub_float("ev.graph", "openWB/")
        # self.pub_float("evseausgelesen", "openWB/")
        # self.pub_float("glattwattbezug", "openWB/")
        # self.pub_float("hausverbrauch", "openWB/")
        self.pub_str("ipaddress", "openWB/system/ip_address")
        # self.pub_float("ledstatus", "openWB/")
        self.pub_int("netzschutz", "openWB/general/grid_protection")
        # self.pub_float("randomSleepValue", "openWB/")
        # self.pub_float("renewmqtt", "openWB/")
        # self.pub_float("rseaktiv", "openWB/")
        # self.pub_float("rsestatus", "openWB/")
        # self.pub_float("schieflast", "openWB/counter/evu/get/unbalanced_load")
        # self.pub_float("u1p3pstat", "openWB/")
        # self.pub_float("uhcounter", "openWB/")
        # self.pub_float("urcounter", "openWB/")
        # self.pub_float("anzahlphasen", "openWB/")
        # self.pub_float("bootinprogress", "openWB/")
        # self.pub_float("execdisplay", "openWB/")
        # self.pub_float("graphtimer", "openWB/")

        #other files
    #     self.pub_float("mqttCp1Configured:-1" \", "openWB/")
    #     self.pub_float("mqttRandomSleepValue:-1" \", "openWB/")
    #     self.pub_float("mqttabschaltuberschuss:-1" \", "openWB/")
    #     self.pub_float("mqttabschaltverzoegerung:-1" \", "openWB/")
    #     self.pub_float("mqttadaptfaktor:-1" \", "openWB/")
    #     self.pub_float("mqttadaptpv:-1" \", "openWB/")
    #     self.pub_float("mqttaktgeladen:-1" \", "openWB/")
    #     self.pub_float("mqttaktgeladens1:-1" \", "openWB/")
    #     self.pub_float("mqttaktgeladens2:-1" \", "openWB/")
    #     self.pub_float("mqttdailychargelp1:-1" \", "openWB/")
    #     self.pub_float("mqttdailychargelp2:-1" \", "openWB/")
    #     self.pub_float("mqttdailychargelp3:-1" \", "openWB/")
    #     self.pub_float("mqttdatenschutzack:-1" \", "openWB/")
    #     self.pub_float("mqttdisplayevumax:-1" \", "openWB/")
    #     self.pub_float("mqttdisplayhausanzeigen:-1" \", "openWB/")
    #     self.pub_float("mqttdisplayhausmax:-1" \", "openWB/")
    #     self.pub_float("mqttdisplaypvmax:-1" \", "openWB/")
    #     self.pub_float("mqttdisplayspeichermax:-1" \", "openWB/")
    #     self.pub_float("mqttdurchslp1:-1" \", "openWB/")
    #     self.pub_float("mqtteinschaltverzoegerung:-1" \", "openWB/")
    #     self.pub_float("mqttetprovideraktiv:-1" \", "openWB/")
    #     self.pub_float("mqttetprovider:notset" \", "openWB/")
    #     self.pub_float("mqttevuglaettungakt:-1" \", "openWB/")
    #     self.pub_float("mqtthausverbrauch:-1" \", "openWB/")
    #     self.pub_float("mqtthausverbrauchstat:-1" \", "openWB/")
    #     self.pub_float("mqttheutegeladen:-1" \", "openWB/")
    #     self.pub_float("mqtthook1_aktiv:-1" \", "openWB/")
    #     self.pub_float("mqtthook2_aktiv:-1" \", "openWB/")
    #     self.pub_float("mqtthook3_aktiv:-1" \", "openWB/")
    #     self.pub_float("mqttlademkwh:-1" \", "openWB/")
    #     self.pub_float("mqttlademkwhlp4:-1" \", "openWB/")
    #     self.pub_float("mqttlademkwhlp5:-1" \", "openWB/")
    #     self.pub_float("mqttlademkwhlp6:-1" \", "openWB/")
    #     self.pub_float("mqttlademkwhlp7:-1" \", "openWB/")
    #     self.pub_float("mqttlademkwhlp8:-1" \", "openWB/")
    #     self.pub_float("mqttlademstat:-1" \", "openWB/")
    #     self.pub_float("mqttlademstatlp4:-1" \", "openWB/")
    #     self.pub_float("mqttlademstatlp5:-1" \", "openWB/")
    #     self.pub_float("mqttlademstatlp6:-1" \", "openWB/")
    #     self.pub_float("mqttlademstatlp7:-1" \", "openWB/")
    #     self.pub_float("mqttlademstatlp8:-1" \", "openWB/")
    #     self.pub_float("mqttlademstats1:-1" \", "openWB/")
    #     self.pub_float("mqttlademstats2:-1" \", "openWB/")
    #     self.pub_float("mqttlastlademodus:-1" \", "openWB/")
    #     self.pub_float("mqttmaximalstromstaerke:-1" \", "openWB/")
    #     self.pub_float("mqttmaxnurpvsoclp1:-1" \", "openWB/")
    #     self.pub_float("mqttmindestuberschuss:-1" \", "openWB/")
    #     self.pub_float("mqttminimalalp2pv:-1" \", "openWB/")
    #     self.pub_float("mqttminimalampv:-1" \", "openWB/")
    #     self.pub_float("mqttminimalapv:-1" \", "openWB/")
    #     self.pub_float("mqttminimalstromstaerke:-1" \", "openWB/")
    #     self.pub_float("mqttminnurpvsocll:-1" \", "openWB/")
    #     self.pub_float("mqttminnurpvsoclp1:-1" \", "openWB/")
    #     self.pub_float("mqttnachtladen:-1" \", "openWB/")
    #     self.pub_float("mqttnachtladens1:-1" \", "openWB/")
    #     self.pub_float("mqttnlakt_minpv:-1" \", "openWB/")
    #     self.pub_float("mqttnlakt_nurpv:-1" \", "openWB/")
    #     self.pub_float("mqttnlakt_sofort:-1" \", "openWB/")
    #     self.pub_float("mqttnlakt_standby:-1" \", "openWB/")
    #     self.pub_float("mqttnurpv70dynact:-1" \", "openWB/")
    #     self.pub_float("mqttnurpv70dynw:-1" \", "openWB/")
    #     self.pub_float("mqttoffsetpv:-1" \", "openWB/")
    #     self.pub_float("mqttpreisjekwh:-1" \", "openWB/")
    #     self.pub_float("mqttpvbezugeinspeisung:-1" \", "openWB/")
    #     self.pub_float("mqttpvwatt:-1" \", "openWB/")
    #     self.pub_float("mqttrestzeitlp1:-1" \", "openWB/")
    #     self.pub_float("mqttrestzeitlp2:-1" \", "openWB/")
    #     self.pub_float("mqttrestzeitlp3:-1" \", "openWB/")
    #     self.pub_float("mqttrfidakt:-1" \", "openWB/")
    #     self.pub_float("mqttsoc1vorhanden:-1" \", "openWB/")
    #     self.pub_float("mqttsoc:-1" \", "openWB/")
    #     self.pub_float("mqttsocvorhanden:-1" \", "openWB/")
    #     self.pub_float("mqttsofortsoclp1:-1" \", "openWB/")
    #     self.pub_float("mqttsofortsoclp2:-1" \", "openWB/")
    #     self.pub_float("mqttsofortsocstatlp1:-1" \", "openWB/")
    #     self.pub_float("mqttsofortsocstatlp2:-1" \", "openWB/")
    #     self.pub_float("mqttspeichermaxwatt:-1" \", "openWB/")
    #     self.pub_float("mqttspeicherpveinbeziehen:-1" \", "openWB/")
    #     self.pub_float("mqttspeicherpvui:-1" \", "openWB/")
    #     self.pub_float("mqttspeichersochystminpv:-1" \", "openWB/")
    #     self.pub_float("mqttspeichersocminpv:-1" \", "openWB/")
    #     self.pub_float("mqttspeichersocnurpv:-1" \", "openWB/")
    #     self.pub_float("mqttspeicherwattnurpv:-1" \", "openWB/")
    #     self.pub_float("mqttstopchargepvatpercentlp1:-1" \", "openWB/")
    #     self.pub_float("mqttstopchargepvatpercentlp2:-1" \", "openWB/")
    #     self.pub_float("mqttstopchargepvpercentagelp1:-1" \", "openWB/")
    #     self.pub_float("mqttstopchargepvpercentagelp2:-1" \", "openWB/")
    #     self.pub_float("mqttu1p3paktiv:-1" \", "openWB/")
    #     self.pub_float("mqttu1p3pminundpv:-1" \", "openWB/")
    #     self.pub_float("mqttu1p3pnl:-1" \", "openWB/")
    #     self.pub_float("mqttu1p3pnurpv:-1" \", "openWB/")
    #     self.pub_float("mqttu1p3psofort:-1" \", "openWB/")
    #     self.pub_float("mqttu1p3pstandby:-1" \", "openWB/")
    #     self.pub_float("mqttupdateinprogress:-1" \", "openWB/")
    #     self.pub_float("mqttverbraucher1_aktiv:-1" \", "openWB/")
    #     self.pub_float("mqttverbraucher1_name:notset" \", "openWB/")
    #     self.pub_float("mqttverbraucher2_aktiv:-1" \", "openWB/")
    #     self.pub_float("mqttverbraucher2_name:notset" \", "openWB/")
    #     self.pub_float("mqttversion:-1" \", "openWB/")
    #     self.pub_float("mqttwattbezug:-1" \", "openWB/")
    #     self.pub_float("mqttwizzarddone:-1"", "openWB/")


pubvars()