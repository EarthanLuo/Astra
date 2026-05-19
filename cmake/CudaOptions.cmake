set(GS_LIVO_CUDA_ARCH "75;80;86;89" CACHE STRING
    "CUDA architectures (compute capabilities)")

function(gs_livo_set_cuda_options target)
    if(NOT CUDAToolkit_FOUND)
        return()
    endif()
    set_target_properties(${target} PROPERTIES
        CUDA_ARCHITECTURES "${GS_LIVO_CUDA_ARCH}"
        CUDA_SEPARABLE_COMPILATION ON
        CUDA_RESOLVE_DEVICE_SYMBOLS OFF
    )

    target_compile_options(${target} PRIVATE
        $<$<COMPILE_LANGUAGE:CUDA>:
            --use_fast_math
            --expt-extended-lambda
            --expt-relaxed-constexpr
        >
    )
endfunction()
