import{C as l}from"./HardwareInstallation-76795b33.js";import{_ as u,u as n,k as d,l as c,G as o,E as r,y as i}from"./vendor-88a3d381.js";import"./vendor-fortawesome-2ab93053.js";import"./index-9eb4725b.js";import"./vendor-bootstrap-6598ffd1.js";import"./vendor-jquery-536f4487.js";import"./vendor-axios-29ac7e52.js";import"./vendor-sortablejs-f1eda7cf.js";import"./dynamic-import-helper-be004503.js";const _={name:"DeviceSmahmInverter",mixins:[l]},f={class:"device-smahm-inverter"};function b(t,e,v,g,h,w){const s=n("openwb-base-heading"),a=n("openwb-base-alert"),m=n("openwb-base-number-input");return d(),c("div",f,[o(s,null,{default:r(()=>e[1]||(e[1]=[i(" Einstellungen für SMA-HM/EM Wechselrichter ")])),_:1}),o(a,{subtype:"info"},{default:r(()=>e[2]||(e[2]=[i(' Dies ist nur die richtige Komponente, wenn ein extra EnergyMeter ausschließlich für die PV-Messung vorhanden ist. Wenn nur ein HomeManager vorhanden ist, muss ein Gerät "SMA Sunny Boy" mit der entsprechenden Wechselrichter-Komponente angelegt werden. ')])),_:1}),o(m,{title:"Seriennummer",required:"","model-value":t.component.configuration.serials,"onUpdate:modelValue":e[0]||(e[0]=p=>t.updateConfiguration(p,"configuration.serials"))},null,8,["model-value"])])}const $=u(_,[["render",b],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/devices/sma/sma_shm/inverter.vue"]]);export{$ as default};
