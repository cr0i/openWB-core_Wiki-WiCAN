import{D as a}from"./HardwareInstallation-76795b33.js";import{_ as p,u as r,k as u,l as d,G as n,E as l,y as m}from"./vendor-88a3d381.js";import"./vendor-fortawesome-2ab93053.js";import"./index-9eb4725b.js";import"./vendor-bootstrap-6598ffd1.js";import"./vendor-jquery-536f4487.js";import"./vendor-axios-29ac7e52.js";import"./vendor-sortablejs-f1eda7cf.js";import"./dynamic-import-helper-be004503.js";const f={name:"DevicePowerfox",mixins:[a]},c={class:"device-powerfox"};function v(o,e,w,_,b,g){const s=r("openwb-base-heading"),i=r("openwb-base-text-input");return u(),d("div",c,[n(s,null,{default:l(()=>e[2]||(e[2]=[m(" Einstellungen für Powerfox ")])),_:1}),n(i,{title:"Benutzername",subtype:"user",required:"","model-value":o.device.configuration.user,"onUpdate:modelValue":e[0]||(e[0]=t=>o.updateConfiguration(t,"configuration.user"))},null,8,["model-value"]),n(i,{title:"Passwort",subtype:"password",required:"","model-value":o.device.configuration.password,"onUpdate:modelValue":e[1]||(e[1]=t=>o.updateConfiguration(t,"configuration.password"))},null,8,["model-value"])])}const P=p(f,[["render",v],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/devices/powerfox/powerfox/device.vue"]]);export{P as default};
