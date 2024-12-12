import{D as u}from"./HardwareInstallation-775406e1.js";import{_ as p,u as i,k as m,l as g,G as t,E as r,y as s}from"./vendor-06e11d0e.js";import"./vendor-fortawesome-3d19d475.js";import"./index-beac009d.js";import"./vendor-bootstrap-4263d7eb.js";import"./vendor-jquery-9fc083b4.js";import"./vendor-axios-22b906fb.js";import"./vendor-sortablejs-0bb60e5b.js";import"./dynamic-import-helper-be004503.js";const b={name:"DeviceSolax",mixins:[u]},f={class:"device-solax"};function v(o,e,c,_,x,w){const d=i("openwb-base-heading"),l=i("openwb-base-text-input"),a=i("openwb-base-number-input");return m(),g("div",f,[t(d,null,{default:r(()=>e[3]||(e[3]=[s(" Einstellungen für Solax ")])),_:1}),t(l,{title:"IP oder Hostname",subtype:"host",required:"","model-value":o.device.configuration.ip_address,"onUpdate:modelValue":e[0]||(e[0]=n=>o.updateConfiguration(n,"configuration.ip_address"))},null,8,["model-value"]),t(a,{title:"Port",min:1,max:65535,placeholder:502,"model-value":o.device.configuration.port,"onUpdate:modelValue":e[1]||(e[1]=n=>o.updateConfiguration(n,"configuration.port"))},{help:r(()=>e[4]||(e[4]=[s(" Standardmäßig ist der Port 502. Dieser sollte nur geändert werden, wenn der Solax Wechselrichter auf einen anderen Port konfiguriert wurde. ")])),_:1},8,["model-value"]),t(a,{title:"Modbus-ID",required:"",min:0,max:255,"model-value":o.device.configuration.modbus_id,"onUpdate:modelValue":e[2]||(e[2]=n=>o.updateConfiguration(n,"configuration.modbus_id"))},{help:r(()=>e[5]||(e[5]=[s(" Laut der Schnittstellenbeschreibung ist die ID bei Solax ab Werk auf 1 gesetzt. Entgegen der Beschreibung ist meist für Gen4 eine ID größer als 2 eingestellt. Bei Gen3 sogar (gegen jeglicher Modbus Standards) eine 0. ")])),_:1},8,["model-value"])])}const $=p(b,[["render",v],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/devices/solax/solax/device.vue"]]);export{$ as default};
