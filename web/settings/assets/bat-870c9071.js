import{C as r}from"./HardwareInstallation-775406e1.js";import{_ as p,u as o,k as m,l as u,G as t,E as l,y as c}from"./vendor-06e11d0e.js";import"./vendor-fortawesome-3d19d475.js";import"./index-beac009d.js";import"./vendor-bootstrap-4263d7eb.js";import"./vendor-jquery-9fc083b4.js";import"./vendor-axios-22b906fb.js";import"./vendor-sortablejs-0bb60e5b.js";import"./dynamic-import-helper-be004503.js";const d={name:"DeviceFemsBat",mixins:[r]},f={class:"device-fems-bat"};function _(n,e,b,v,g,C){const i=o("openwb-base-heading"),s=o("openwb-base-number-input");return m(),u("div",f,[t(i,null,{default:l(()=>e[1]||(e[1]=[c(" Einstellungen für openEMS, Fenecon FEMS, CENTURIO 10, Kaco Hy-Control Batteriespeicher ")])),_:1}),t(s,{title:"Anzahl der verbauten Speicher",required:"",min:1,max:2,"model-value":n.component.configuration.num,"onUpdate:modelValue":e[0]||(e[0]=a=>n.updateConfiguration(a,"configuration.num"))},null,8,["model-value"])])}const N=p(d,[["render",_],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/devices/fems/fems/bat.vue"]]);export{N as default};
