# Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.

#############
# Macro ExpandDependencyTree
#    Expands a dependency tree into a linear list by recursively traversing the tree.
#    Useful to do stuff with the libs without worrying about transitive dependencies.
#    libraries - List of all top level dependencies
#    libraries_list - List containing all dependencies in the tree
macro(ExpandDependencyTree libraries libraries_list_var config)
  foreach(lib ${libraries})
    #Avoid duplicates
    #Note: an IF(MATCHES) fails because of special characters in lib
    list (FIND ${libraries_list_var} "${lib}" _index)
    if (${_index} EQUAL -1)
      #Add to list
      set(${libraries_list_var} ${${libraries_list_var}} ${lib})

      if (TARGET ${lib})
        # recursion for dependencies

        # INTERFACE_LINK_LIBRARIES usually points to targets
        get_property(dependencies TARGET ${lib} PROPERTY INTERFACE_LINK_LIBRARIES)
        if (dependencies)
          ExpandDependencyTree("${dependencies}" ${libraries_list_var} ${config})
        endif()

        get_property(IS_IMPORTED TARGET ${lib} PROPERTY IMPORTED)
        if (${IS_IMPORTED})
          # IMPORTED_LINK_INTERFACE_LIBRARIES_${config_dst} usually points to specific files
          get_property(is_set TARGET ${lib} PROPERTY IMPORTED_LINK_INTERFACE_LIBRARIES_${config} SET)
          if(is_set)
            get_property(dependencies TARGET ${lib} PROPERTY IMPORTED_LINK_INTERFACE_LIBRARIES_${config})
            if (dependencies)
              ExpandDependencyTree("${dependencies}" ${libraries_list_var} ${config})
            endif()
          endif()

          # IMPORTED_LINK_INTERFACE_LIBRARIES usually points to specific files
          get_property(is_set TARGET ${lib} PROPERTY IMPORTED_LINK_INTERFACE_LIBRARIES SET)
          if(is_set)
            get_property(dependencies TARGET ${lib} PROPERTY IMPORTED_LINK_INTERFACE_LIBRARIES)
            if (dependencies)
              ExpandDependencyTree("${dependencies}" ${libraries_list_var} ${config})
            endif()
          endif()
        endif()
      endif()
    endif()
  endforeach()
endmacro()
