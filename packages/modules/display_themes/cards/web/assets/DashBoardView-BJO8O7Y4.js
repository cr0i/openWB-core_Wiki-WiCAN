import{u as g}from"./index-CtySFcBa.js";import{D as S}from"./DashBoardCard-BterAPyf.js";import{S as q,C as k}from"./ChargePointPlugBadge-Bsi0zWop.js";import{l as P,d as v,F as w,e as x,g as B,h as D,i as I}from"./vendor-fortawesome-C68yAli-.js";import{_ as h}from"./vendor-inkline-Ce5aFAnW.js";import{o as c,l as i,n as o,s as d,k as m,x as p,q as e,i as _,e as H}from"./vendor-CZ1MGz7j.js";P.add(v);const L={name:"GridCard",components:{DashBoardCard:S,SparkLine:q,FontAwesomeIcon:w},props:{},data(){return{mqttStore:g()}}};function E(l,r,f,u,t,C){const a=e("font-awesome-icon"),n=e("spark-line"),s=e("dash-board-card");return c(),i(s,{color:"danger"},{headerLeft:o(()=>[d(a,{"fixed-width":"",icon:["fas","fa-gauge-high"]}),r[0]||(r[0]=m(" EVU "))]),headerRight:o(()=>[m(p(t.mqttStore.getGridPower()),1)]),default:o(()=>[d(n,{color:"var(--color--danger)","color-negative":"var(--color--success)",data:t.mqttStore.getGridPowerChartData},null,8,["data"])]),_:1})}const V=h(L,[["render",E]]);P.add(x);const G={name:"BatteryCard",components:{DashBoardCard:S,SparkLine:q,FontAwesomeIcon:w},props:{},data(){return{mqttStore:g()}}};function R(l,r,f,u,t,C){const a=e("font-awesome-icon"),n=e("spark-line"),s=e("dash-board-card");return t.mqttStore.getBatteryConfigured?(c(),i(s,{key:0,color:"warning"},{headerLeft:o(()=>[d(a,{"fixed-width":"",icon:["fas","fa-car-battery"]}),r[0]||(r[0]=m(" Speicher "))]),headerRight:o(()=>[m(p(t.mqttStore.getBatterySoc())+" / "+p(t.mqttStore.getBatteryPower()),1)]),default:o(()=>[d(n,{color:"var(--color--warning)",data:t.mqttStore.getBatteryPowerChartData,"soc-data":t.mqttStore.getBatterySocChartData},null,8,["data","soc-data"])]),_:1})):_("",!0)}const $=h(G,[["render",R]]);P.add(B);const N={name:"InverterCard",components:{DashBoardCard:S,SparkLine:q,FontAwesomeIcon:w},props:{},data(){return{mqttStore:g()}}};function F(l,r,f,u,t,C){const a=e("font-awesome-icon"),n=e("spark-line"),s=e("dash-board-card");return t.mqttStore.getPvConfigured?(c(),i(s,{key:0,color:"success"},{headerLeft:o(()=>[d(a,{"fixed-width":"",icon:["fas","fa-solar-panel"]}),r[0]||(r[0]=m(" PV "))]),headerRight:o(()=>[m(p(t.mqttStore.getPvPower()),1)]),default:o(()=>[d(n,{color:"var(--color--success)",data:t.mqttStore.getPvPowerChartData,inverted:!0},null,8,["data"])]),_:1})):_("",!0)}const A=h(N,[["render",F]]);P.add(D);const M={name:"HomeCard",components:{DashBoardCard:S,SparkLine:q,FontAwesomeIcon:w},props:{},data(){return{mqttStore:g()}}};function T(l,r,f,u,t,C){const a=e("font-awesome-icon"),n=e("spark-line"),s=e("dash-board-card");return c(),i(s,{color:"light"},{headerLeft:o(()=>[d(a,{"fixed-width":"",icon:["fas","fa-home"]}),r[0]||(r[0]=m(" Hausverbrauch "))]),headerRight:o(()=>[m(p(t.mqttStore.getHomePower()),1)]),default:o(()=>[d(n,{color:"var(--color--light)",data:t.mqttStore.getHomePowerChartData},null,8,["data"])]),_:1})}const U=h(M,[["render",T]]);P.add(I);const j={name:"ChargePointsCard",components:{DashBoardCard:S,SparkLine:q,FontAwesomeIcon:w,ChargePointPlugBadge:k},props:{},data(){return{mqttStore:g()}}};function z(l,r,f,u,t,C){const a=e("font-awesome-icon"),n=e("charge-point-plug-badge"),s=e("spark-line"),y=e("dash-board-card");return t.mqttStore.getChargePointIds.length>0?(c(),i(y,{key:0,color:"primary"},{headerLeft:o(()=>[d(a,{"fixed-width":"",icon:["fas","fa-charging-station"]}),m(" "+p(t.mqttStore.getChargePointIds.length==1?t.mqttStore.getChargePointName(t.mqttStore.getChargePointIds[0]):"Ladepunkte"),1)]),headerRight:o(()=>[m(p(t.mqttStore.getChargePointIds.length==1?t.mqttStore.getChargePointPower(t.mqttStore.getChargePointIds[0]):t.mqttStore.getChargePointSumPower())+" ",1),d(n,{"charge-point-id":t.mqttStore.getChargePointIds,"show-energy-charged":!1},null,8,["charge-point-id"])]),default:o(()=>[d(s,{color:"var(--color--primary)",data:t.mqttStore.getChargePointIds.length==1?t.mqttStore.getChargePointPowerChartData(t.mqttStore.getChargePointIds[0]):t.mqttStore.getChargePointSumPowerChartData},null,8,["data"])]),_:1})):_("",!0)}const J=h(j,[["render",z]]),K={name:"DashboardView",components:{GridCard:V,HomeCard:U,BatteryCard:$,InverterCard:A,ChargePointsCard:J},props:{changesLocked:{required:!1,type:Boolean,default:!1}},data(){return{mqttStore:g()}}},O={class:"dash-board-card-wrapper"};function Q(l,r,f,u,t,C){const a=e("grid-card"),n=e("home-card"),s=e("battery-card"),y=e("inverter-card"),b=e("charge-points-card");return c(),H("div",O,[t.mqttStore.getGridCardEnabled?(c(),i(a,{key:0})):_("",!0),t.mqttStore.getHomeCardEnabled?(c(),i(n,{key:1})):_("",!0),t.mqttStore.getBatteryCardEnabled?(c(),i(s,{key:2})):_("",!0),t.mqttStore.getPvCardEnabled?(c(),i(y,{key:3})):_("",!0),t.mqttStore.getChargePointsCardEnabled?(c(),i(b,{key:4})):_("",!0)])}const ot=h(K,[["render",Q],["__scopeId","data-v-2085947b"]]);export{ot as default};
