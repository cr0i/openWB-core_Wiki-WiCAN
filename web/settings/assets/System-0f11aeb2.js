import{l as W,am as q,t as A,ad as C,a2 as S,an as $,F as U}from"./vendor-fortawesome-ef7cec82.js";import{C as z}from"./index-9a36dc3a.js";import{_ as E,u as b,l as c,m as w,x as B,E as o,z as y,G as s,A as l,B as i,q as _,N as F,M as j,F as T}from"./vendor-3e356662.js";import"./vendor-bootstrap-0c918764.js";import"./vendor-jquery-dd8b8694.js";import"./vendor-axios-c995a1bc.js";import"./vendor-sortablejs-6ccb40db.js";W.add(q,A,C,S,$);const x={name:"OpenwbSystemView",components:{FontAwesomeIcon:U},mixins:[z],props:{installAssistantActive:{type:Boolean,required:!1,default:!1}},emits:["sendCommand"],data(){return{mqttTopicsToSubscribe:["openWB/system/optionBackup","openWB/system/current_commit","openWB/system/current_branch_commit","openWB/system/current_missing_commits","openWB/system/available_branches","openWB/system/current_branch","openWB/system/version","openWB/system/serial_number","openWB/system/ip_address","openWB/system/mac_address"],warningAcknowledged:!1,selectedTag:"*HEAD*"}},computed:{updateAvailable(){return this.$store.state.mqtt["openWB/system/current_branch_commit"]&&this.$store.state.mqtt["openWB/system/current_branch_commit"]!=this.$store.state.mqtt["openWB/system/current_commit"]},releaseChangeValid(){return this.$store.state.mqtt["openWB/system/current_branch"]in this.$store.state.mqtt["openWB/system/available_branches"]&&"tags"in this.$store.state.mqtt["openWB/system/available_branches"][this.$store.state.mqtt["openWB/system/current_branch"]]&&this.selectedTag in this.$store.state.mqtt["openWB/system/available_branches"][this.$store.state.mqtt["openWB/system/current_branch"]].tags}},methods:{sendSystemCommand(t,e={}){this.$emit("sendCommand",{command:t,data:e})},getBranchGroups(){const t="Release",e="Beta",u="master",v=[t,e],p=(a,d)=>a.value==d.value?0:a.value==t?-1:d.value==t?1:a.value==e?-1:d.value==e?1:a.value==u?-1:d.value==u||a.value>d.value?1:a.value<d.value?-1:0;var r=this.$store.state.mqtt["openWB/system/available_branches"],f=[{label:"Allgemein",options:[]},{label:"Alpha-Zweig",options:[]},{label:"Experimentell",options:[]}];if(r!==void 0){var m=0;for(const[a,d]of Object.entries(r))v.includes(a)?m=0:a==u?m=1:m=2,f[m].options.push({value:a,text:a+" ("+d.commit+")"});f[0].options.sort(p),f[1].options.sort(p)}return f},getBranchTagOptions(){if(!(this.$store.state.mqtt["openWB/system/current_branch"]in this.$store.state.mqtt["openWB/system/available_branches"]))return[];var t=this.$store.state.mqtt["openWB/system/available_branches"][this.$store.state.mqtt["openWB/system/current_branch"]].tags,e=[];if(t!==void 0)for(const[u,v]of Object.entries(t))e.unshift({value:u,text:v});return e},updateConfiguration(t,e){console.debug("updateConfiguration",t,e),this.updateState(t,e.value,e.object)},systemUpdate(){this.sendSystemCommand("systemUpdate",{}),this.$store.commit("storeLocal",{name:"reloadRequired",value:!0})},switchBranch(){this.sendSystemCommand("systemUpdate",{branch:this.$store.state.mqtt["openWB/system/current_branch"],tag:this.selectedTag}),this.$store.commit("storeLocal",{name:"reloadRequired",value:!0})}}},D={class:"system"},L={key:1},N={name:"versionInfoForm"},I={class:"missing-commits"},R={class:"row justify-content-center"},Z={class:"col-md-4 d-flex py-1 justify-content-center"},O={class:"col-md-4 d-flex py-1 justify-content-center"},P={key:0,name:"powerForm"},G={class:"row justify-content-center"},M={class:"col-md-4 d-flex py-1 justify-content-center"},H={class:"col-md-4 d-flex py-1 justify-content-center"},J={key:1,name:"releaseChangeForm"},K={class:"row justify-content-center"},Q={class:"col-md-4 d-flex py-1 justify-content-center"};function X(t,e,u,v,p,r){const f=b("openwb-base-button-group-input"),m=b("openwb-base-alert"),a=b("openwb-base-text-input"),d=b("openwb-base-card"),g=b("font-awesome-icon"),h=b("openwb-base-click-button"),k=b("openwb-base-select-input");return c(),w("div",D,[u.installAssistantActive?y("",!0):(c(),B(m,{key:0,subtype:"danger"},{default:o(()=>[e[15]||(e[15]=l("h2",null,"Achtung!",-1)),e[16]||(e[16]=l("p",null," Vor allen Aktionen auf dieser Seite ist sicherzustellen, dass kein Ladevorgang aktiv ist! Zur Sicherheit bitte zusätzlich alle Fahrzeuge von der Ladestation / den Ladestationen abstecken! ",-1)),s(f,{modelValue:p.warningAcknowledged,"onUpdate:modelValue":e[0]||(e[0]=n=>p.warningAcknowledged=n),title:"Ich habe die Warnung verstanden",buttons:[{buttonValue:!1,text:"Nein",class:"btn-outline-danger"},{buttonValue:!0,text:"Ja",class:"btn-outline-success"}]},null,8,["modelValue"])]),_:1})),p.warningAcknowledged||u.installAssistantActive?(c(),w("div",L,[s(d,{title:"System Information",subtype:"info",collapsible:!0,collapsed:!0},{default:o(()=>[s(a,{modelValue:t.$store.state.mqtt["openWB/system/serial_number"],"onUpdate:modelValue":e[1]||(e[1]=n=>t.$store.state.mqtt["openWB/system/serial_number"]=n),title:"Seriennummer",readonly:""},null,8,["modelValue"]),s(a,{modelValue:t.$store.state.mqtt["openWB/system/ip_address"],"onUpdate:modelValue":e[2]||(e[2]=n=>t.$store.state.mqtt["openWB/system/ip_address"]=n),title:"IP-Adresse",readonly:""},null,8,["modelValue"]),s(a,{modelValue:t.$store.state.mqtt["openWB/system/mac_address"],"onUpdate:modelValue":e[3]||(e[3]=n=>t.$store.state.mqtt["openWB/system/mac_address"]=n),title:"MAC-Adresse",readonly:""},null,8,["modelValue"])]),_:1}),l("form",N,[s(d,{title:"Versions-Informationen / Aktualisierung",subtype:"success",collapsible:!0,collapsed:!u.installAssistantActive},{footer:o(()=>[l("div",R,[l("div",Z,[s(h,{class:"btn-info",onButtonClicked:e[8]||(e[8]=n=>r.sendSystemCommand("systemFetchVersions"))},{default:o(()=>[e[19]||(e[19]=i(" Informationen aktualisieren ")),s(g,{"fixed-width":"",icon:["fas","download"]})]),_:1})]),l("div",O,[s(h,{class:_(r.updateAvailable?"btn-success clickable":"btn-outline-success"),disabled:!r.updateAvailable,onButtonClicked:e[9]||(e[9]=n=>r.systemUpdate())},{default:o(()=>[e[20]||(e[20]=i(" Update ")),s(g,{"fixed-width":"",icon:["fas","arrow-alt-circle-up"]})]),_:1},8,["class","disabled"])])])]),default:o(()=>[s(a,{modelValue:t.$store.state.mqtt["openWB/system/current_branch"],"onUpdate:modelValue":e[4]||(e[4]=n=>t.$store.state.mqtt["openWB/system/current_branch"]=n),title:"Entwicklungszweig",readonly:""},null,8,["modelValue"]),s(a,{modelValue:t.$store.state.mqtt["openWB/system/version"],"onUpdate:modelValue":e[5]||(e[5]=n=>t.$store.state.mqtt["openWB/system/version"]=n),title:"Bezeichnung",readonly:""},null,8,["modelValue"]),s(a,{modelValue:t.$store.state.mqtt["openWB/system/current_commit"],"onUpdate:modelValue":e[6]||(e[6]=n=>t.$store.state.mqtt["openWB/system/current_commit"]=n),title:"installierte Version",readonly:"",class:_(r.updateAvailable?"text-danger":"text-success")},null,8,["modelValue","class"]),s(a,{modelValue:t.$store.state.mqtt["openWB/system/current_branch_commit"],"onUpdate:modelValue":e[7]||(e[7]=n=>t.$store.state.mqtt["openWB/system/current_branch_commit"]=n),title:"aktuellste Version",readonly:""},null,8,["modelValue"]),r.updateAvailable?(c(),B(d,{key:0,title:"Änderungen",subtype:"info",collapsible:!0,collapsed:!0},{default:o(()=>[l("ul",I,[(c(!0),w(F,null,j(t.$store.state.mqtt["openWB/system/current_missing_commits"],(n,V)=>(c(),w("li",{key:V},T(n),1))),128))])]),_:1})):y("",!0),s(m,{subtype:"danger"},{default:o(()=>e[17]||(e[17]=[i(" Nach einem Update wird die Ladestation direkt neu gestartet! Es werden alle eventuell vorhandenen lokalen Änderungen am Programmcode mit dem Update verworfen! ")])),_:1}),t.$store.state.mqtt["openWB/system/current_branch"]!="Release"?(c(),B(m,{key:1,subtype:"danger"},{default:o(()=>e[18]||(e[18]=[i(' Du bist nicht auf dem für den normalen Gebrauch empfohlenen Entwicklungszweig "Release". Wir empfehlen, auf diesen Zweig zu wechseln, sobald dort eine neue Version verfügbar ist.'),l("br",null,null,-1),i(" Bevor ein Update angestoßen wird, sollte immer eine Sicherung erstellt werden! Es kann zwar wieder auf eine ältere Version gewechselt werden, jedoch ist nicht sichergestellt, dass es dabei keine Probleme gibt. Gerade wenn das Datenformat in der neuen Version angepasst wurde, wird eine ältere damit Fehler produzieren. ")])),_:1})):y("",!0)]),_:1},8,["collapsed"])]),u.installAssistantActive?y("",!0):(c(),w("form",P,[s(d,{title:"Betrieb",collapsible:!0,collapsed:!0},{footer:o(()=>[l("div",G,[l("div",M,[s(h,{class:"btn-warning",onButtonClicked:e[10]||(e[10]=n=>r.sendSystemCommand("systemReboot"))},{default:o(()=>[e[22]||(e[22]=i(" Neustart ")),s(g,{"fixed-width":"",icon:["fas","undo"]})]),_:1})]),l("div",H,[s(h,{class:"btn-danger",onButtonClicked:e[11]||(e[11]=n=>r.sendSystemCommand("systemShutdown"))},{default:o(()=>[e[23]||(e[23]=i(" Ausschalten ")),s(g,{"fixed-width":"",icon:["fas","power-off"]})]),_:1})])])]),default:o(()=>[s(m,{subtype:"danger"},{default:o(()=>e[21]||(e[21]=[i(" Wenn die Ladestation ausgeschaltet wird, muss sie komplett spannungsfrei geschaltet werden. Erst beim erneuten Zuschalten der Spannung fährt das System wieder hoch. ")])),_:1})]),_:1})])),u.installAssistantActive?y("",!0):(c(),w("form",J,[s(d,{title:"Entwicklungszweig",subtype:"danger",collapsible:!0,collapsed:!0},{footer:o(()=>[l("div",K,[l("div",Q,[s(h,{class:_(r.releaseChangeValid?"btn-danger clickable":"btn-outline-danger"),disabled:!r.releaseChangeValid,onButtonClicked:e[14]||(e[14]=n=>r.switchBranch())},{default:o(()=>[s(g,{"fixed-width":"",icon:["fas","skull-crossbones"]}),e[26]||(e[26]=i(" Branch und Tag wechseln ")),s(g,{"fixed-width":"",icon:["fas","skull-crossbones"]})]),_:1},8,["class","disabled"])])])]),default:o(()=>[s(m,{subtype:"danger"},{default:o(()=>e[24]||(e[24]=[i(" Nach einem Wechsel wird die Ladestation direkt neu gestartet! Es werden alle lokalen Änderungen mit dem Wechsel verworfen! ")])),_:1}),s(m,{subtype:"warning"},{default:o(()=>e[25]||(e[25]=[i(" Bevor auf einen neuen Entwicklungszweig gewechselt wird sollte immer eine Sicherung erstellt werden! Es kann zwar wieder auf eine ältere Version gewechselt werden, jedoch ist nicht sichergestellt, dass es dabei keine Probleme gibt. Gerade wenn das Datenformat in der neuen Version angepasst wurde, wird eine ältere damit Fehler produzieren."),l("br",null,null,-1),i(' Für den normalen Betrieb wird der Zweig "Release" empfohlen. Der Softwarestand wurde ausgiebig getestet, sodass ein Fehlverhalten relativ unwahrscheinlich ist.'),l("br",null,null,-1),i(' Der "Beta" Zweig beinhaltet Vorabversionen, bei denen die Entwicklung soweit abgeschlossen ist. Die enthaltenen Anpassungen wurden rudimentär getestet, können aber durchaus noch Fehler enthalten.'),l("br",null,null,-1),i(' Die aktuelle Softwareentwicklung findet im Zweig "master" statt. Die enthaltenen Anpassungen sind teilweise noch nicht getestet und enthalten potentiell Fehler.'),l("br",null,null,-1),i(' Einträge, die mit "feature" beginnen, sind experimentelle Entwicklungszweige, die nicht für den allgemeinen Gebrauch gedacht sind. ')])),_:1}),s(k,{title:"Entwicklungszweig",groups:r.getBranchGroups(),"model-value":t.$store.state.mqtt["openWB/system/current_branch"],"onUpdate:modelValue":e[12]||(e[12]=n=>t.updateState("openWB/system/current_branch",n))},null,8,["groups","model-value"]),s(k,{modelValue:p.selectedTag,"onUpdate:modelValue":e[13]||(e[13]=n=>p.selectedTag=n),title:"Tag",options:r.getBranchTagOptions()},null,8,["modelValue","options"])]),_:1})]))])):y("",!0)])}const le=E(x,[["render",X],["__scopeId","data-v-059847f6"],["__file","/opt/openWB-dev/openwb-ui-settings/src/views/System.vue"]]);export{le as default};
