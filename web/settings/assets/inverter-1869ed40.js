import{C as a}from"./HardwareInstallation-775406e1.js";import{_ as p,u as t,k as u,l,G as n,E as c,y as m}from"./vendor-06e11d0e.js";import"./vendor-fortawesome-3d19d475.js";import"./index-beac009d.js";import"./vendor-bootstrap-4263d7eb.js";import"./vendor-jquery-9fc083b4.js";import"./vendor-axios-22b906fb.js";import"./vendor-sortablejs-0bb60e5b.js";import"./dynamic-import-helper-be004503.js";const d={name:"DeviceKostalPikoInverter",mixins:[a]},_={class:"device-kostal-piko-inverter"};function f(o,e,b,v,g,k){const i=t("openwb-base-heading"),r=t("openwb-base-button-group-input");return u(),l("div",_,[n(i,null,{default:c(()=>e[1]||(e[1]=[m(" Einstellungen für Kostal Piko Wechselrichter ")])),_:1}),n(r,{title:"Speicher",buttons:[{buttonValue:!1,text:"nicht vorhanden"},{buttonValue:!0,text:"vorhanden"}],"model-value":o.component.configuration.bat_configured,"onUpdate:modelValue":e[0]||(e[0]=s=>o.updateConfiguration(s,"configuration.bat_configured"))},null,8,["model-value"])])}const N=p(d,[["render",f],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/devices/kostal/kostal_piko/inverter.vue"]]);export{N as default};
