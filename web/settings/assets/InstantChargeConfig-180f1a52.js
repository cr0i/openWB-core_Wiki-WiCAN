import{C as l}from"./index-9eb4725b.js";import{_ as b,u as o,k as a,l as i,x as _,G as s,E as r,y as u}from"./vendor-88a3d381.js";import"./vendor-fortawesome-2ab93053.js";import"./vendor-bootstrap-6598ffd1.js";import"./vendor-jquery-536f4487.js";import"./vendor-axios-29ac7e52.js";import"./vendor-sortablejs-f1eda7cf.js";const f={name:"OpenwbInstantChargeConfigView",mixins:[l],emits:["save","reset","defaults"],data(){return{mqttTopicsToSubscribe:["openWB/general/extern","openWB/general/chargemode_config/instant_charging/phases_to_use"]}}},h={class:"instantChargeConfig"},c={name:"instantChargeConfigForm"},v={key:0},w={key:1};function C(t,e,$,B,k,V){const p=o("openwb-base-alert"),m=o("openwb-base-button-group-input"),g=o("openwb-base-card"),d=o("openwb-base-submit-buttons");return a(),i("div",h,[_("form",c,[s(g,{title:"Phasenumschaltung"},{default:r(()=>[t.$store.state.mqtt["openWB/general/extern"]===!0?(a(),i("div",v,[s(p,{subtype:"info"},{default:r(()=>e[4]||(e[4]=[u(' Diese Einstellungen sind nicht verfügbar, solange sich diese openWB im Steuerungsmodus "secondary" befindet. ')])),_:1})])):(a(),i("div",w,[s(m,{title:"Anzahl Phasen",buttons:[{buttonValue:1,text:"1"},{buttonValue:3,text:"Maximum"}],"model-value":t.$store.state.mqtt["openWB/general/chargemode_config/instant_charging/phases_to_use"],"onUpdate:modelValue":e[0]||(e[0]=n=>t.updateState("openWB/general/chargemode_config/instant_charging/phases_to_use",n))},{help:r(()=>e[5]||(e[5]=[u(' Hier kann eingestellt werden, ob Ladevorgänge im Modus "Sofortladen" mit nur einer Phase oder dem möglichen Maximum in Abhängigkeit der "Ladepunkt"- und "Fahrzeug"-Einstellungen durchgeführt werden. Voraussetzung ist die verbaute Umschaltmöglichkeit zwischen 1- und 3-phasig (s.g. 1p3p). ')])),_:1},8,["model-value"])]))]),_:1}),s(d,{"form-name":"instantChargeConfigForm",onSave:e[1]||(e[1]=n=>t.$emit("save")),onReset:e[2]||(e[2]=n=>t.$emit("reset")),onDefaults:e[3]||(e[3]=n=>t.$emit("defaults"))})])])}const F=b(f,[["render",C],["__file","/opt/openWB-dev/openwb-ui-settings/src/views/InstantChargeConfig.vue"]]);export{F as default};
