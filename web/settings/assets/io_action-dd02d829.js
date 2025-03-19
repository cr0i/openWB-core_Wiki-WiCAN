import{a as c}from"./IoConfig-9865782e.js";import{O as g}from"./OpenwbIoSinglePattern-c023bf2c.js";import{_ as v,u,l as f,m as b,G as p,A as _,E as w,B as h,N as D}from"./vendor-3e356662.js";import"./vendor-fortawesome-ef7cec82.js";import"./index-9a36dc3a.js";import"./vendor-bootstrap-0c918764.js";import"./vendor-jquery-dd8b8694.js";import"./vendor-axios-c995a1bc.js";import"./vendor-sortablejs-6ccb40db.js";import"./dynamic-import-helper-be004503.js";const V={name:"IoActionDimmingDirectControl",components:{OpenwbIoSinglePattern:g},mixins:[c],computed:{value:{get(){return this.ioAction.configuration.input_pattern},set(t){this.updateConfiguration(t,"configuration.input_pattern")}},ioDevicesOutputOptions(){let t=[];return this.availableIoDevices.forEach(e=>{let n=[];Object.keys(e==null?void 0:e.output.digital).forEach(o=>{n.push({text:`${o}`,value:{type:"io",id:e.id,digital_output:o}})}),n.length>0&&t.push({label:e.name,options:n})}),t},availableDevices(){return[{label:"Ladepunkte",options:this.availableChargePoints.map(t=>({value:{type:"cp",id:t.value},text:t.text}))}].concat(this.ioDevicesOutputOptions)}}};function O(t,e,n,o,k,i){var l,s,r;const d=u("openwb-io-single-pattern"),m=u("openwb-base-select-input");return f(),b(D,null,[p(d,{modelValue:i.value,"onUpdate:modelValue":e[0]||(e[0]=a=>i.value=a),"digital-inputs":(s=(l=t.ioDevice)==null?void 0:l.input)==null?void 0:s.digital,"io-device":t.ioDevice},null,8,["modelValue","digital-inputs","io-device"]),e[3]||(e[3]=_("hr",null,null,-1)),p(m,{title:"Verhalten anwenden auf...","not-selected":"Bitte auswählen","empty-value":[],groups:i.availableDevices,multiple:"",required:"",disabled:!(Object.keys(i.value[0].input_matrix).length>0),"model-value":(r=t.ioAction)==null?void 0:r.configuration.devices,"onUpdate:modelValue":e[1]||(e[1]=a=>t.updateConfiguration(a,"configuration.devices"))},{help:w(()=>e[2]||(e[2]=[h(" Bitte die Ladepunkte und/oder digitalen Ausgänge auswählen, auf welche das Verhalten angewendet werden soll. Es können mehrere Einträge ausgewählt werden. ")])),_:1},8,["groups","disabled","model-value"])],64)}const L=v(V,[["render",O],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/io_actions/controllable_consumers/dimming_direct_control/io_action.vue"]]);export{L as default};
