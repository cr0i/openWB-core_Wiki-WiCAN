import{C as a}from"./HardwareInstallation-76795b33.js";import{_ as p,u as n,k as d,l as m,G as t,E as l,y as u}from"./vendor-88a3d381.js";import"./vendor-fortawesome-2ab93053.js";import"./index-9eb4725b.js";import"./vendor-bootstrap-6598ffd1.js";import"./vendor-jquery-536f4487.js";import"./vendor-axios-29ac7e52.js";import"./vendor-sortablejs-f1eda7cf.js";import"./dynamic-import-helper-be004503.js";const c={name:"DeviceSolaredgeInverter",mixins:[a]},_={class:"device-solaredge-inverter"};function f(o,e,g,b,v,w){const r=n("openwb-base-heading"),i=n("openwb-base-number-input");return d(),m("div",_,[t(r,null,{default:l(()=>e[1]||(e[1]=[u(" Einstellungen für SolarEdge Wechselrichter ")])),_:1}),t(i,{title:"SolarEdge-Geräte-ID",required:"","model-value":o.component.configuration.modbus_id,min:"1",max:"255","onUpdate:modelValue":e[0]||(e[0]=s=>o.updateConfiguration(s,"configuration.modbus_id"))},null,8,["model-value"])])}const D=p(c,[["render",f],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/devices/solaredge/solaredge/inverter.vue"]]);export{D as default};
