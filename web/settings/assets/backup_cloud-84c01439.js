import{B as r}from"./DataManagement-3ab650ed.js";import{_ as p,u,l as i,m as l,G as d,E as c,y as o,x as n}from"./vendor-0c15df0c.js";import"./vendor-fortawesome-231ff303.js";import"./index-6ffbdc7e.js";import"./vendor-bootstrap-83e2d5a1.js";import"./vendor-jquery-84e2bf4a.js";import"./vendor-axios-c9d2afa0.js";import"./vendor-sortablejs-1a751103.js";import"./dynamic-import-helper-be004503.js";const m={name:"BackupCloudNfs",mixins:[r]},f={class:"backup-cloud-nfs"};function _(t,e,b,k,v,x){const a=u("openwb-base-text-input");return i(),l("div",f,[d(a,{title:"Pfad zum NFS Share",pattern:"^([^/: ]+):(\\/[^/: ]+)+$",required:"","model-value":t.backupCloud.configuration.nfs_share,"onUpdate:modelValue":e[0]||(e[0]=s=>t.updateConfiguration(s,"configuration.nfs_share"))},{help:c(()=>e[1]||(e[1]=[o(" 1. IP basierter Pfad: 1.2.3.4:/pfad/zum/nfs-share"),n("br",null,null,-1),o(" 2. Host (FQDN) basierter Pfad: backupserver.domain.net:/pfad/zum/nfs-share"),n("br",null,null,-1),o(" Bitte entgegen der Syntax das Protokoll nicht davor schreiben. Falsch wäre, “nfs://” davor zu setzen. ")])),_:1},8,["model-value"])])}const V=p(m,[["render",_],["__file","/opt/openWB-dev/openwb-ui-settings/src/components/backup_clouds/nfs/backup_cloud.vue"]]);export{V as default};
