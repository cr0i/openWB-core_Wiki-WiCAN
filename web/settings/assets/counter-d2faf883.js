import{C as p}from"./HardwareInstallation-76795b33.js";import{_ as u,u as n,k as a,l as m,G as t,E as d,y as l}from"./vendor-88a3d381.js";import"./vendor-fortawesome-2ab93053.js";import"./index-9eb4725b.js";import"./vendor-bootstrap-6598ffd1.js";import"./vendor-jquery-536f4487.js";import"./vendor-axios-29ac7e52.js";import"./vendor-sortablejs-f1eda7cf.js";import"./dynamic-import-helper-be004503.js";const c={name:"DeviceSunnyBoyCounter",mixins:[p]},_={class:"device-sunnyboy-counter"};function b(o,e,f,v,y,g){const i=n("openwb-base-heading"),s=n("openwb-base-number-input");return a(),m("div",_,[t(i,null,{default:d(()=>e[1]||(e[1]=[l(" Einstellungen für SMA Sunny Boy/Tripower Zähler ")])),_:1}),t(s,{title:"Modbus ID",required:"","model-value":o.component.configuration.modbus_id,min:"1",max:"255","onUpdate:modelValue":e[0]||(e[0]=r=>o.updateConfiguration(r,"configuration.modbus_id"))},null,8,["model-value"])])}const V=u(c,[["render",b],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/devices/sma/sma_sunny_boy/counter.vue"]]);export{V as default};
