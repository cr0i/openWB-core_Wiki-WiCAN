import{C as a}from"./HardwareInstallation-76795b33.js";import{_ as p,u as t,k as u,l as d,G as n,E as m,y as l}from"./vendor-88a3d381.js";import"./vendor-fortawesome-2ab93053.js";import"./index-9eb4725b.js";import"./vendor-bootstrap-6598ffd1.js";import"./vendor-jquery-536f4487.js";import"./vendor-axios-29ac7e52.js";import"./vendor-sortablejs-f1eda7cf.js";import"./dynamic-import-helper-be004503.js";const c={name:"DeviceStuderBat",mixins:[a]},_={class:"device-studer-bat"};function b(o,e,f,v,g,w){const i=t("openwb-base-heading"),s=t("openwb-base-number-input");return u(),d("div",_,[n(i,null,{default:m(()=>e[1]||(e[1]=[l(" Einstellungen für Studer Batteriespeicher ")])),_:1}),n(s,{title:"Modbus ID",required:"","model-value":o.component.configuration.modbus_id,min:"1",max:"255","onUpdate:modelValue":e[0]||(e[0]=r=>o.updateConfiguration(r,"configuration.modbus_id"))},null,8,["model-value"])])}const M=p(c,[["render",b],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/devices/studer/studer/bat.vue"]]);export{M as default};
