#
#  Do not edit this file.  This file is generated from 
#  package.bld.  Any modifications to this file will be 
#  overwritten whenever makefiles are re-generated.
#

unexport MAKEFILE_LIST
MK_NOGENDEPS := $(filter clean,$(MAKECMDGOALS))
override PKGDIR = configPkg
XDCINCS = -I. -I$(strip $(subst ;, -I,$(subst $(space),\$(space),$(XPKGPATH))))
XDCCFGDIR = package/cfg/

#
# The following dependencies ensure package.mak is rebuilt
# in the event that some included BOM script changes.
#
ifneq (clean,$(MAKECMDGOALS))
C:/ti/xdctools_3_51_01_18_core/packages/xdc/utils.js:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/utils.js
C:/ti/xdctools_3_51_01_18_core/packages/xdc/xdc.tci:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/xdc.tci
C:/ti/xdctools_3_51_01_18_core/packages/xdc/template.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/template.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/om2.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/om2.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/xmlgen.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/xmlgen.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/xmlgen2.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/xmlgen2.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/Warnings.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/Warnings.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/IPackage.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/IPackage.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/package.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/package.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/services/global/Clock.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/services/global/Clock.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/services/global/Trace.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/services/global/Trace.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/bld.js:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/bld.js
C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/BuildEnvironment.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/BuildEnvironment.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/PackageContents.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/PackageContents.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/_gen.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/_gen.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/Library.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/Library.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/Executable.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/Executable.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/Repository.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/Repository.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/Configuration.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/Configuration.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/Script.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/Script.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/Manifest.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/Manifest.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/Utils.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/Utils.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/ITarget.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/ITarget.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/ITarget2.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/ITarget2.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/ITarget3.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/ITarget3.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/ITargetFilter.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/ITargetFilter.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/package.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/bld/package.xs
package.mak: config.bld
C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/kernel/tirtos/packages/iar/targets/arm/ITarget.xs:
package.mak: C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/kernel/tirtos/packages/iar/targets/arm/ITarget.xs
C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/kernel/tirtos/packages/iar/targets/arm/package.xs:
package.mak: C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/kernel/tirtos/packages/iar/targets/arm/package.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/services/io/File.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/services/io/File.xs
C:/ti/xdctools_3_51_01_18_core/packages/xdc/services/io/package.xs:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/services/io/package.xs
package.mak: package.bld
C:/ti/xdctools_3_51_01_18_core/packages/xdc/tools/configuro/template/compiler.opt.xdt:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/tools/configuro/template/compiler.opt.xdt
C:/ti/xdctools_3_51_01_18_core/packages/xdc/tools/configuro/template/compiler.defs.xdt:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/tools/configuro/template/compiler.defs.xdt
C:/ti/xdctools_3_51_01_18_core/packages/xdc/tools/configuro/template/custom.mak.exe.xdt:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/tools/configuro/template/custom.mak.exe.xdt
C:/ti/xdctools_3_51_01_18_core/packages/xdc/tools/configuro/template/package.xs.xdt:
package.mak: C:/ti/xdctools_3_51_01_18_core/packages/xdc/tools/configuro/template/package.xs.xdt
endif

iar.targets.arm.M4F.rootDir ?= C:/PROGRA~2/IARSYS~1/EMBEDD~1.0/arm
iar.targets.arm.packageBase ?= C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/kernel/tirtos/packages/iar/targets/arm/
.PRECIOUS: $(XDCCFGDIR)/%.orm4f
.PHONY: all,rm4f .dlls,rm4f .executables,rm4f test,rm4f
all,rm4f: .executables,rm4f
.executables,rm4f: .libraries,rm4f
.executables,rm4f: .dlls,rm4f
.dlls,rm4f: .libraries,rm4f
.libraries,rm4f: .interfaces
	@$(RM) $@
	@$(TOUCH) "$@"

.help::
	@$(ECHO) xdc test,rm4f
	@$(ECHO) xdc .executables,rm4f
	@$(ECHO) xdc .libraries,rm4f
	@$(ECHO) xdc .dlls,rm4f


all: .executables 
.executables: .libraries .dlls
.libraries: .interfaces

PKGCFGS := $(wildcard package.xs) package/build.cfg
.interfaces: package/package.xdc.inc package/package.defs.h package.xdc $(PKGCFGS)

-include package/package.xdc.dep
package/%.xdc.inc package/%_configPkg.c package/%.defs.h: %.xdc $(PKGCFGS)
	@$(MSG) generating interfaces for package configPkg" (because $@ is older than $(firstword $?))" ...
	$(XSRUN) -f xdc/services/intern/cmd/build.xs $(MK_IDLOPTS) -m package/package.xdc.dep -i package/package.xdc.inc package.xdc

.dlls,rm4f .dlls: ble_release.prm4f

-include package/cfg/ble_release_prm4f.mak
-include package/cfg/ble_release_prm4f.cfg.mak
ifeq (,$(MK_NOGENDEPS))
-include package/cfg/ble_release_prm4f.dep
endif
ble_release.prm4f: package/cfg/ble_release_prm4f.xdl
	@


ifeq (,$(wildcard .libraries,rm4f))
ble_release.prm4f package/cfg/ble_release_prm4f.c: .libraries,rm4f
endif

package/cfg/ble_release_prm4f.c package/cfg/ble_release_prm4f.h package/cfg/ble_release_prm4f.xdl: override _PROG_NAME := ble_release.xrm4f
package/cfg/ble_release_prm4f.c: package/cfg/ble_release_prm4f.cfg
package/cfg/ble_release_prm4f.xdc.inc: package/cfg/ble_release_prm4f.xdl
package/cfg/ble_release_prm4f.xdl package/cfg/ble_release_prm4f.c: .interfaces

clean:: clean,rm4f
	-$(RM) package/cfg/ble_release_prm4f.cfg
	-$(RM) package/cfg/ble_release_prm4f.dep
	-$(RM) package/cfg/ble_release_prm4f.c
	-$(RM) package/cfg/ble_release_prm4f.xdc.inc

clean,rm4f::
	-$(RM) ble_release.prm4f
.executables,rm4f .executables: ble_release.xrm4f

ble_release.xrm4f: |ble_release.prm4f

-include package/cfg/ble_release.xrm4f.mak
ble_release.xrm4f: package/cfg/ble_release_prm4f.orm4f 
	$(RM) $@
	@$(MSG) lnkrm4f $@ ...
	$(RM) $(XDCCFGDIR)/$@.map
	LC_ALL=C $(iar.targets.arm.M4F.rootDir)/bin/ilinkarm -o C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_CyclicVoltammetry_3/tirtos/iar/app/FlashROM_Release/Exe/ble5_simple_peripheral_cc26x2r1lp_app_FlashROM_Release.out --config_def CC2642=1 --config_def PAGE_ALIGN=1 --config_def FLASH_ROM_BUILD=2 --map C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_CyclicVoltammetry_3/tirtos/iar/app/FlashROM_Release/List/ble5_simple_peripheral_cc26x2r1lp_app_FlashROM_Release.map --config C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_CyclicVoltammetry_3/tirtos/iar/app/../../../../../../../../source/ti/ble5stack/common/cc26xx/iar/cc26xx_app_and_stack_agama.icf --keep __vector_table -f C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_CyclicVoltammetry_3/tirtos/iar/app/../config/configPkg/linker.cmd -f C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_CyclicVoltammetry_3/tirtos/iar/app/../config/lib_linker.cmd --semihosting C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_CyclicVoltammetry_3/tirtos/iar/app/../config/ble_r2.symbols C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_CyclicVoltammetry_3/tirtos/iar/app/../../../../../../../../source/ti/devices/cc13x2_cc26x2/driverlib/bin/iar/driverlib.lib C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_CyclicVoltammetry_3/tirtos/iar/app/../../../../../../../../kernel/tirtos/packages/ti/dpl/lib/dpl_cc26x2.arm4f C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_CyclicVoltammetry_3/tirtos/iar/app/../../../../../../../../source/ti/drivers/lib/drivers_cc26x2.arm4f C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_CyclicVoltammetry_3/tirtos/iar/app/../../../../../../../../source/ti/drivers/rf/lib/rf_multiMode_cc26x2.arm4f C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_CyclicVoltammetry_3/tirtos/iar/app/../../../../../../../../source/ti/display/lib/display.arm4f C:/ti/simplelink_cc13x2_26x2_sdk_2_40_00_81/examples/rtos/CC26X2R1_LAUNCHXL/ble5stack/CC2642_CyclicVoltammetry_3/tirtos/iar/app/../../../../../../../../source/ti/grlib/lib/iar/m4f/grlib.a --entry __iar_program_start --vfe --text_out locale --silent -o $@ package/cfg/ble_release_prm4f.orm4f -f package/cfg/ble_release_prm4f.xdl  --semihosting=iar_breakpoint  --cpu=Cortex-M4F --map $(XDCCFGDIR)/$@.map  --redirect _Printf=_PrintfSmall --redirect _Scanf=_ScanfSmall 
	

ble_release.test test,rm4f test: ble_release.xrm4f.test

ble_release.xrm4f.test:: ble_release.xrm4f
ifeq (,$(_TESTLEVEL))
	@$(MAKE) -R -r --no-print-directory -f $(XDCROOT)/packages/xdc/bld/xdc.mak _TESTLEVEL=1 ble_release.xrm4f.test
else
	@$(MSG) running $<  ...
	$(call EXEC.ble_release.xrm4f, ) 
endif

clean,rm4f::
	-$(RM) $(wildcard .tmp,ble_release.xrm4f,*)


clean:: clean,rm4f

clean,rm4f::
	-$(RM) ble_release.xrm4f
%,copy:
	@$(if $<,,$(MSG) don\'t know how to build $*; exit 1)
	@$(MSG) cp $< $@
	$(RM) $@
	$(CP) $< $@
ble_release_prm4f.orm4f,copy : package/cfg/ble_release_prm4f.orm4f
ble_release_prm4f.srm4f,copy : package/cfg/ble_release_prm4f.srm4f

$(XDCCFGDIR)%.c $(XDCCFGDIR)%.h $(XDCCFGDIR)%.xdl: $(XDCCFGDIR)%.cfg $(XDCROOT)/packages/xdc/cfg/Main.xs | .interfaces
	@$(MSG) "configuring $(_PROG_NAME) from $< ..."
	$(CONFIG) $(_PROG_XSOPTS) xdc.cfg $(_PROG_NAME) $(XDCCFGDIR)$*.cfg $(XDCCFGDIR)$*

.PHONY: release,configPkg
ifeq (,$(MK_NOGENDEPS))
-include package/rel/configPkg.tar.dep
endif
package/rel/configPkg/configPkg/package/package.rel.xml: package/package.bld.xml
package/rel/configPkg/configPkg/package/package.rel.xml: package/build.cfg
package/rel/configPkg/configPkg/package/package.rel.xml: package/package.xdc.inc
package/rel/configPkg/configPkg/package/package.rel.xml: .force
	@$(MSG) generating external release references $@ ...
	$(XS) $(JSENV) -f $(XDCROOT)/packages/xdc/bld/rel.js $(MK_RELOPTS) . $@

configPkg.tar: package/rel/configPkg.xdc.inc package/rel/configPkg/configPkg/package/package.rel.xml
	@$(MSG) making release file $@ "(because of $(firstword $?))" ...
	-$(RM) $@
	$(call MKRELTAR,package/rel/configPkg.xdc.inc,package/rel/configPkg.tar.dep)


release release,configPkg: all configPkg.tar
clean:: .clean
	-$(RM) configPkg.tar
	-$(RM) package/rel/configPkg.xdc.inc
	-$(RM) package/rel/configPkg.tar.dep

clean:: .clean
	-$(RM) .libraries $(wildcard .libraries,*)
clean:: 
	-$(RM) .dlls $(wildcard .dlls,*)
#
# The following clean rule removes user specified
# generated files or directories.
#

ifneq (clean,$(MAKECMDGOALS))
ifeq (,$(wildcard package))
    $(shell $(MKDIR) package)
endif
ifeq (,$(wildcard package/cfg))
    $(shell $(MKDIR) package/cfg)
endif
ifeq (,$(wildcard package/lib))
    $(shell $(MKDIR) package/lib)
endif
ifeq (,$(wildcard package/rel))
    $(shell $(MKDIR) package/rel)
endif
ifeq (,$(wildcard package/internal))
    $(shell $(MKDIR) package/internal)
endif
endif
clean::
	-$(RMDIR) package

include custom.mak
