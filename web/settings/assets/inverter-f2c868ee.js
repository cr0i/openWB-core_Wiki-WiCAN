import{C as p}from"./HardwareInstallation-76795b33.js";import{_ as a,u as n,k as m,l as u,G as t,E as d,y as l}from"./vendor-88a3d381.js";import"./vendor-fortawesome-2ab93053.js";import"./index-9eb4725b.js";import"./vendor-bootstrap-6598ffd1.js";import"./vendor-jquery-536f4487.js";import"./vendor-axios-29ac7e52.js";import"./vendor-sortablejs-f1eda7cf.js";import"./dynamic-import-helper-be004503.js";const _={name:"DeviceFoxEssInverter",mixins:[p]},c={class:"device-fox_ess-inverter"};function f(o,e,b,v,x,g){const s=n("openwb-base-heading"),i=n("openwb-base-number-input");return m(),u("div",c,[t(s,null,{default:d(()=>e[1]||(e[1]=[l(" Einstellungen für FoxEss Wechselrichter ")])),_:1}),t(i,{title:"Modbus ID",required:"","model-value":o.component.configuration.modbus_id,min:"1",max:"255","onUpdate:modelValue":e[0]||(e[0]=r=>o.updateConfiguration(r,"configuration.modbus_id"))},null,8,["model-value"])])}const F=a(_,[["render",f],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/devices/fox_ess/fox_ess/inverter.vue"]]);export{F as default};
