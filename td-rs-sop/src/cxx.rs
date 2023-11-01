#![allow(non_snake_case)]
use crate::{Sop, SopOutput, SopVboOutput};
use autocxx::prelude::*;
use autocxx::subclass::*;

use std::ffi::CString;

use std::pin::Pin;
use td_rs_base::{param::ParameterManager, OperatorInputs, NodeInfo};

include_cpp! {
    #include "SOP_CPlusPlusBase.h"
    #include "RustSopPlugin.h"
    safety!(unsafe)
    extern_cpp_type!("TD::OP_ParameterManager", td_rs_base::cxx::OP_ParameterManager)
    extern_cpp_type!("TD::OP_String", td_rs_base::cxx::OP_String)
    extern_cpp_type!("TD::OP_InfoDATSize", td_rs_base::cxx::OP_InfoDATSize)
    extern_cpp_type!("TD::OP_InfoCHOPChan", td_rs_base::cxx::OP_InfoCHOPChan)
    extern_cpp_type!("TD::OP_Inputs", td_rs_base::cxx::OP_Inputs)
    generate_pod!("TD::SOP_GeneralInfo")
    generate_pod!("TD::SOP_PluginInfo")
    generate!("TD::SOP_Output")
    generate!("TD::SOP_VBOOutput")
    extern_cpp_type!("TD::Vector", td_rs_base::cxx::Vector)
    extern_cpp_type!("TD::Position", td_rs_base::cxx::Position)
    extern_cpp_type!("TD::Color", td_rs_base::cxx::Color)
    extern_cpp_type!("TD::TexCoord", td_rs_base::cxx::TexCoord)
    extern_cpp_type!("TD::BoundingBox", td_rs_base::cxx::BoundingBox)
    extern_cpp_type!("TD::SOP_CustomAttribData", td_rs_base::cxx::SOP_CustomAttribData)
    extern_cpp_type!("TD::SOP_CustomAttribInfo", td_rs_base::cxx::SOP_CustomAttribInfo)
    extern_cpp_type!("TD::OP_CustomOPInfo", td_rs_base::cxx::OP_CustomOPInfo)
    pod!("TD::OP_CustomOPInfo")
    generate_pod!("TD::SOP_Winding")
}

pub use autocxx::c_void;
pub use td_rs_base::cxx::*;
pub use ffi::TD::*;
pub use ffi::*;

extern "C" {
    fn sop_new_impl(info: NodeInfo) -> Box<dyn Sop>;
}

#[subclass(superclass("RustSopPlugin"))]
pub struct RustSopPluginImpl {
    inner: Box<dyn Sop>,
}

#[no_mangle]
extern "C" fn sop_new(info: &'static OP_NodeInfo) -> *mut RustSopPluginImplCpp {
    unsafe {
        let info = NodeInfo::new(info);
        RustSopPluginImpl::new_cpp_owned(RustSopPluginImpl {
            inner: sop_new_impl(info),
            cpp_peer: CppSubclassCppPeerHolder::Empty,
        }).into_raw()
    }
}

impl RustSopPlugin_methods for RustSopPluginImpl {
    fn getGeneralInfo(&mut self, mut info: Pin<&mut SOP_GeneralInfo>, inputs: &OP_Inputs) {
        let input = OperatorInputs::new(inputs);
        if let Some(params) = self.inner.params_mut() {
            params.update(&input.params());
        }
        let gen_info = self.inner.general_info(&input);
        info.cookEveryFrame = gen_info.cook_every_frame;
        info.cookEveryFrameIfAsked = gen_info.cook_every_frame_if_asked;
        info.directToGPU = gen_info.direct_to_gpu;
        info.winding = SOP_Winding::CCW;
    }

    fn execute(&mut self, outputs: Pin<&mut SOP_Output>, inputs: &OP_Inputs) {
        let input = OperatorInputs::new(inputs);
        let mut output = SopOutput::new(outputs);
        if let Some(params) = self.inner.params_mut() {
            params.update(&input.params());
        }
        self.inner.execute(&mut output, &input);
    }

    fn executeVBO(&mut self, output: Pin<&mut SOP_VBOOutput>, inputs: &OP_Inputs) {
        let input = OperatorInputs::new(inputs);
        let mut output = SopVboOutput::new(output);
        self.inner.execute_vbo(&mut output, &input);
    }

    fn getNumInfoCHOPChans(&mut self) -> i32 {
        if let Some(info_chop) = self.inner.info_chop() {
            info_chop.size() as i32
        } else {
            0
        }
    }

    fn getInfoCHOPChan(&mut self, index: i32, name: Pin<&mut OP_String>, mut value: Pin<&mut f32>) {
        if let Some(info_chop) = self.inner.info_chop() {
            let (info_name, info_value) = info_chop.channel(index as usize);
            unsafe {
                let new_string = CString::new(info_name.as_str()).unwrap();
                let new_string_ptr = new_string.as_ptr();
                name.setString(new_string_ptr);
            }
            value.set(info_value);
        }
    }

    fn getInfoDATSize(&mut self, mut info: Pin<&mut OP_InfoDATSize>) -> bool {
        if let Some(info_dat) = self.inner.info_dat() {
            let (rows, cols) = info_dat.size();
            info.rows = rows as i32;
            info.cols = cols as i32;
            true
        } else {
            false
        }
    }

    fn getInfoDATEntry(&mut self, index: i32, entryIndex: i32, entry: Pin<&mut OP_String>) {
        if let Some(info_dat) = self.inner.info_dat() {
            let entry_str = info_dat.entry(index as usize, entryIndex as usize);
            if entry_str.is_empty() {
                return;
            }
            unsafe {
                let new_string = CString::new(entry_str.as_str()).unwrap();
                let new_string_ptr = new_string.as_ptr();
                entry.setString(new_string_ptr);
            }
        }
    }

    fn getWarningString(&mut self, warning: Pin<&mut OP_String>) {
        unsafe {
            let new_string = CString::new(self.inner.warning()).unwrap();
            let new_string_ptr = new_string.as_ptr();
            warning.setString(new_string_ptr);
        }
    }

    fn getErrorString(&mut self, error: Pin<&mut OP_String>) {
        unsafe {
            let new_string = CString::new(self.inner.error()).unwrap();
            let new_string_ptr = new_string.as_ptr();
            error.setString(new_string_ptr);
        }
    }

    fn getInfoPopupString(&mut self, info: Pin<&mut OP_String>) {
        unsafe {
            let new_string = CString::new(self.inner.info()).unwrap();
            let new_string_ptr = new_string.as_ptr();
            info.setString(new_string_ptr);
        }
    }

    fn setupParameters(&mut self, manager: Pin<&mut OP_ParameterManager>) {
        let params = self.inner.params_mut();
        if let Some(params) = params {
            let mut manager = ParameterManager::new(manager);
            params.register(&mut manager);
        }
    }

    unsafe fn pulsePressed(&mut self, name: *const std::ffi::c_char) {
        self.inner
            .pulse_pressed(std::ffi::CStr::from_ptr(name).to_str().unwrap());
    }
}
