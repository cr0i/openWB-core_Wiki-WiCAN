import{D as d}from"./HardwareInstallation-774805b0.js";import{_ as u,u as i,l as p,m as l,G as t,E as m,y as f}from"./vendor-0c15df0c.js";import"./vendor-fortawesome-231ff303.js";import"./index-6ffbdc7e.js";import"./vendor-bootstrap-83e2d5a1.js";import"./vendor-jquery-84e2bf4a.js";import"./vendor-axios-c9d2afa0.js";import"./vendor-sortablejs-1a751103.js";import"./dynamic-import-helper-be004503.js";const b={name:"DeviceJanitza",mixins:[d]},v={class:"device-janitza"};function _(o,e,c,g,w,C){const r=i("openwb-base-heading"),s=i("openwb-base-text-input"),a=i("openwb-base-number-input");return p(),l("div",v,[t(r,null,{default:m(()=>e[3]||(e[3]=[f(" Einstellungen für Janitza ")])),_:1}),t(s,{title:"IP oder Hostname",subtype:"host",required:"","model-value":o.device.configuration.ip_address,"onUpdate:modelValue":e[0]||(e[0]=n=>o.updateConfiguration(n,"configuration.ip_address"))},null,8,["model-value"]),t(a,{title:"Port",required:"",min:1,max:65535,"model-value":o.device.configuration.port,"onUpdate:modelValue":e[1]||(e[1]=n=>o.updateConfiguration(n,"configuration.port"))},null,8,["model-value"]),t(a,{title:"Modbus ID",required:"","model-value":o.device.configuration.modbus_id,min:"1",max:"255","onUpdate:modelValue":e[2]||(e[2]=n=>o.updateConfiguration(n,"configuration.modbus_id"))},null,8,["model-value"])])}const U=u(b,[["render",_],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/devices/janitza/janitza/device.vue"]]);export{U as default};
