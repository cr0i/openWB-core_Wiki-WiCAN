import{D as p}from"./HardwareInstallation-775406e1.js";import{_ as a,u as o,k as d,l,G as t,E as u,y as m}from"./vendor-06e11d0e.js";import"./vendor-fortawesome-3d19d475.js";import"./index-beac009d.js";import"./vendor-bootstrap-4263d7eb.js";import"./vendor-jquery-9fc083b4.js";import"./vendor-axios-22b906fb.js";import"./vendor-sortablejs-0bb60e5b.js";import"./dynamic-import-helper-be004503.js";const c={name:"DeviceBenning",mixins:[p]},_={class:"device-benning"};function f(n,e,b,g,v,x){const i=o("openwb-base-heading"),s=o("openwb-base-text-input");return d(),l("div",_,[t(i,null,{default:u(()=>e[1]||(e[1]=[m("Einstellungen für Benning")])),_:1}),t(s,{title:"IP oder Hostname",subtype:"host",required:"","model-value":n.device.configuration.url,"onUpdate:modelValue":e[0]||(e[0]=r=>n.updateConfiguration(r,"configuration.url"))},null,8,["model-value"])])}const y=a(c,[["render",f],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/devices/benning/benning/device.vue"]]);export{y as default};
