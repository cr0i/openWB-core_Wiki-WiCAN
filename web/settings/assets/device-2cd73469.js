import{D as l}from"./HardwareInstallation-76795b33.js";import{_ as r,u as a,k as d,l as p,G as i,E as u,y as m}from"./vendor-88a3d381.js";import"./vendor-fortawesome-2ab93053.js";import"./index-9eb4725b.js";import"./vendor-bootstrap-6598ffd1.js";import"./vendor-jquery-536f4487.js";import"./vendor-axios-29ac7e52.js";import"./vendor-sortablejs-f1eda7cf.js";import"./dynamic-import-helper-be004503.js";const f={name:"DeviceTesla",mixins:[l]},v={class:"device-tesla"};function c(o,e,g,b,_,w){const s=a("openwb-base-heading"),n=a("openwb-base-text-input");return d(),p("div",v,[i(s,null,{default:u(()=>e[3]||(e[3]=[m(" Einstellungen für Tesla ")])),_:1}),i(n,{title:"IP oder Hostname",subtype:"host",required:"","model-value":o.device.configuration.ip_address,"onUpdate:modelValue":e[0]||(e[0]=t=>o.updateConfiguration(t,"configuration.ip_address"))},null,8,["model-value"]),i(n,{title:"E-Mail",subtype:"email",required:"","model-value":o.device.configuration.email,"onUpdate:modelValue":e[1]||(e[1]=t=>o.updateConfiguration(t,"configuration.email"))},null,8,["model-value"]),i(n,{title:"Passwort",subtype:"password",required:"","model-value":o.device.configuration.password,"onUpdate:modelValue":e[2]||(e[2]=t=>o.updateConfiguration(t,"configuration.password"))},null,8,["model-value"])])}const D=r(f,[["render",c],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/devices/tesla/tesla/device.vue"]]);export{D as default};
