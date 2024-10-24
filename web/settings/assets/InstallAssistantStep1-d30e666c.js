import{C as m}from"./index-9eb4725b.js";import p from"./InstallAssistantStepTemplate-acd71d0f.js";import{D as l}from"./DataManagement-32ccaa74.js";import{_ as d,u as a,k as u,z as g,E as o,x as f,G as v}from"./vendor-88a3d381.js";import"./vendor-fortawesome-2ab93053.js";import"./vendor-bootstrap-6598ffd1.js";import"./vendor-jquery-536f4487.js";import"./vendor-axios-29ac7e52.js";import"./vendor-sortablejs-f1eda7cf.js";import"./dynamic-import-helper-be004503.js";const A={name:"InstallAssistantStep1",components:{InstallAssistantStepTemplate:p,DataManagement:l},mixins:[m],emits:["save","reset","defaults","sendCommand","switchPage","endAssistant"],data:()=>({mqttTopicsToSubscribe:[]}),methods:{nextPage(){this.$emit("switchPage",2)},previousPage(){this.$emit("switchPage",0)},endAssistant(){this.$emit("endAssistant")}}};function P(t,e,c,S,w,n){const i=a("DataManagement"),r=a("InstallAssistantStepTemplate");return u(),g(r,{title:"1. Datensicherung der bestehenden Konfiguration",onNextPage:n.nextPage,onPreviousPage:n.previousPage,onEndAssistant:n.endAssistant},{help:o(()=>e[4]||(e[4]=[f("p",null," Wir empfehlen an dieser Stelle eine Sicherung der openWB zu erstellen, auf welche später zurückgegriffen werden kann, insbesondere, wenn die openWB schon konfiguriert war und der Assistent nun erneut ausgeführt wird. ",-1)])),content:o(()=>[v(i,{"install-assistant-active":!0,"show-backup-cloud-section":!1,onSendCommand:e[0]||(e[0]=s=>t.$emit("sendCommand",s)),onSave:e[1]||(e[1]=s=>t.$emit("save")),onReset:e[2]||(e[2]=s=>t.$emit("reset")),onDefaults:e[3]||(e[3]=s=>t.$emit("defaults"))})]),_:1},8,["onNextPage","onPreviousPage","onEndAssistant"])}const N=d(A,[["render",P],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/install_assistant/InstallAssistantStep1.vue"]]);export{N as default};
