import{D as l}from"./HardwareInstallation-76795b33.js";import{_ as m,u as n,k as b,l as _,G as t,E as r,y as s}from"./vendor-88a3d381.js";import"./vendor-fortawesome-2ab93053.js";import"./index-9eb4725b.js";import"./vendor-bootstrap-6598ffd1.js";import"./vendor-jquery-536f4487.js";import"./vendor-axios-29ac7e52.js";import"./vendor-sortablejs-f1eda7cf.js";import"./dynamic-import-helper-be004503.js";const c={name:"DeviceSunnyBoy",mixins:[l]},f={class:"device-sunnyboy"};function v(o,e,w,y,g,x){const a=n("openwb-base-heading"),p=n("openwb-base-alert"),d=n("openwb-base-text-input"),u=n("openwb-base-number-input");return b(),_("div",f,[t(a,null,{default:r(()=>e[2]||(e[2]=[s(" Einstellungen für SMA Sunny Boy/Tripower ")])),_:1}),t(p,{subtype:"info"},{default:r(()=>e[3]||(e[3]=[s(' ModbusTCP muss entweder direkt am Wechselrichter, per Sunny Portal oder über das Tool "Sunny Explorer" aktiviert werden. Dies ist standardmäßig deaktiviert. ')])),_:1}),t(d,{title:"IP oder Hostname",subtype:"host",required:"","model-value":o.device.configuration.ip_address,"onUpdate:modelValue":e[0]||(e[0]=i=>o.updateConfiguration(i,"configuration.ip_address"))},null,8,["model-value"]),t(u,{title:"Port",required:"",min:1,max:65535,"model-value":o.device.configuration.port,"onUpdate:modelValue":e[1]||(e[1]=i=>o.updateConfiguration(i,"configuration.port"))},null,8,["model-value"])])}const $=m(c,[["render",v],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/devices/sma/sma_sunny_boy/device.vue"]]);export{$ as default};
