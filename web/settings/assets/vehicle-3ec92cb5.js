import{V as s}from"./VehicleConfig-b1ef38e0.js";import{_ as d,u as a,k as p,l as m,G as t,E as u,y as n,x as i}from"./vendor-88a3d381.js";import"./vendor-fortawesome-2ab93053.js";import"./index-9eb4725b.js";import"./vendor-bootstrap-6598ffd1.js";import"./vendor-jquery-536f4487.js";import"./vendor-axios-29ac7e52.js";import"./vendor-sortablejs-f1eda7cf.js";import"./dynamic-import-helper-be004503.js";const v={name:"VehicleSocOVMS",mixins:[s]},f={class:"vehicle-soc-ovms"};function g(o,e,V,b,S,w){const l=a("openwb-base-text-input");return p(),m("div",f,[t(l,{title:"Server URL",required:"","model-value":o.vehicle.configuration.server_url,"onUpdate:modelValue":e[0]||(e[0]=r=>o.updateConfiguration(r,"configuration.server_url"))},{help:u(()=>e[4]||(e[4]=[n(" Die URL (incl. Port) des API des OVMS-Servers, an dem das OVMS-Modul angemeldet ist, z.B. "),i("br",null,null,-1),n(" https://ovms.dexters-web.de:6869"),i("br",null,null,-1),n(" oder"),i("br",null,null,-1),n(" https://api.openvehicles.com:6869"),i("br",null,null,-1),n(" oder ein custom server"),i("br",null,null,-1)])),_:1},8,["model-value"]),t(l,{title:"Benutzername",required:"",subtype:"user","model-value":o.vehicle.configuration.user_id,"onUpdate:modelValue":e[1]||(e[1]=r=>o.updateConfiguration(r,"configuration.user_id"))},{help:u(()=>e[5]||(e[5]=[n(" Der Benutzername für die Anmeldung am benutzen OVMS-Servers. ")])),_:1},8,["model-value"]),t(l,{title:"Kennwort",required:"",subtype:"password","model-value":o.vehicle.configuration.password,"onUpdate:modelValue":e[2]||(e[2]=r=>o.updateConfiguration(r,"configuration.password"))},{help:u(()=>e[6]||(e[6]=[n(" Das Passwort für die Anmeldung am OVMS-Server. ")])),_:1},8,["model-value"]),t(l,{title:"Vehicle Id",required:"","model-value":o.vehicle.configuration.vehicleId,"onUpdate:modelValue":e[3]||(e[3]=r=>o.updateConfiguration(r,"configuration.vehicleId"))},{help:u(()=>e[7]||(e[7]=[n(" Die Id des Fahrzeugs im OVMS, auch Module Id genannt. ")])),_:1},8,["model-value"])])}const q=d(v,[["render",g],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/vehicles/ovms/vehicle.vue"]]);export{q as default};
