import{D as r}from"./HardwareInstallation-775406e1.js";import{_ as a,u as t,k as d,l as u,G as n,E as l,y as m}from"./vendor-06e11d0e.js";import"./vendor-fortawesome-3d19d475.js";import"./index-beac009d.js";import"./vendor-bootstrap-4263d7eb.js";import"./vendor-jquery-9fc083b4.js";import"./vendor-axios-22b906fb.js";import"./vendor-sortablejs-0bb60e5b.js";import"./dynamic-import-helper-be004503.js";const c={name:"DeviceOpenDTU",mixins:[r]},_={class:"device-opendtu"};function f(o,e,v,b,g,x){const i=t("openwb-base-heading"),s=t("openwb-base-text-input");return d(),u("div",_,[n(i,null,{default:l(()=>e[1]||(e[1]=[m("Einstellungen für OpenDTU")])),_:1}),n(s,{title:"IP oder Hostname",subtype:"host",required:"","model-value":o.device.configuration.url,"onUpdate:modelValue":e[0]||(e[0]=p=>o.updateConfiguration(p,"configuration.url"))},null,8,["model-value"])])}const V=a(c,[["render",f],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/devices/opendtu/opendtu/device.vue"]]);export{V as default};
