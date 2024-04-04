import{_ as m,q as n,k as _,l as b,B as t,M as u,x as r,u as f,y as g}from"./vendor-c0ce7727.js";import"./vendor-sortablejs-4bc62cd6.js";const v={name:"DeviceOpenwbFlexConsumptionCounter",emits:["update:configuration"],props:{configuration:{type:Object,required:!0},deviceId:{default:void 0},componentId:{required:!0}},methods:{updateConfiguration(o,e=void 0){this.$emit("update:configuration",{value:o,object:e})}}},w={class:"device-openwb-flex-consumption-counter"},h={class:"small"};function x(o,e,s,B,D,a){const d=n("openwb-base-heading"),l=n("openwb-base-alert"),p=n("openwb-base-select-input"),c=n("openwb-base-number-input");return _(),b("div",w,[t(d,null,{default:u(()=>[r(" Einstellungen für openWB-Flex Verbrauchszähler "),f("span",h,"(Modul: "+g(o.$options.name)+")",1)]),_:1}),t(l,{subtype:"info"},{default:u(()=>[r(" Bei saldierenden Zählern (B23) werden die Zählerstände für Einspeisung und Bezug aus dem Zähler ausgelesen. Bei Zählern, die nicht saldierend arbeiten (SDM120, SDM630, SDM72D-M), wird der Zählerstand für die Einspeisung berechnet. ")]),_:1}),t(p,{title:"Zählermodell",notSelected:"Bitte auswählen",options:[{value:"sdm120",text:"SDM120"},{value:"sdm630",text:"SDM630/SDM72D-M"},{value:"b23",text:"B23"}],"model-value":s.configuration.version,"onUpdate:modelValue":e[0]||(e[0]=i=>a.updateConfiguration(i,"configuration.version"))},null,8,["model-value"]),t(c,{title:"Modbus-ID",required:"",min:1,max:255,"model-value":s.configuration.id,"onUpdate:modelValue":e[1]||(e[1]=i=>a.updateConfiguration(i,"configuration.id"))},null,8,["model-value"])])}const C=m(v,[["render",x],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/devices/openwb_flex/consumption_counter.vue"]]);export{C as default};
