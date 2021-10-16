/*****************************************************************************
 * Xidi
 *   DirectInput interface for XInput controllers.
 *****************************************************************************
 * Authored by Samuel Grossman
 * Copyright (c) 2016-2021
 *************************************************************************//**
 * @file MapperBuilder.h
 *   Declaration of functionality for building new mapper objects piece-wise
 *   at runtime.
 *****************************************************************************/

#pragma once

#include "ApiWindows.h"
#include "ControllerTypes.h"
#include "ElementMapper.h"
#include "Mapper.h"

#include <map>
#include <memory>
#include <optional>
#include <string_view>


namespace Xidi
{
    namespace Controller
    {
        /// Encapsulates all functionality for managing a set of partially-built mappers and constructing them into full mapper objects.
        class MapperBuilder
        {
        public:
            // -------- TYPE DEFINITIONS ----------------------------------- //

            /// Maps from element map index to element mapper object.
            /// Used within a blueprint to describe the element map to be created when the mapper is built.
            typedef std::map<unsigned int, std::unique_ptr<IElementMapper>> TElementMapSpec;


            /// Holds a description about how to build a single mapper object.
            struct SBlueprint
            {
                /// Name of the mapper that will be used as a template.
                /// Templates are useful for building new mappers based on other mappers.
                /// If no template is specified then the mapper is being built completely from scratch.
                /// A mapper with this as its name is resolved at mapper build time, not at name setting time.
                std::wstring_view templateName;

                /// Holds changes to be applied to the template when the mapper is being built.
                /// For mappers being built from scratch without a template, holds all of the controller element mappers.
                TElementMapSpec changesFromTemplate;

                /// Flag for specifying if an attempt was made to build the mapper described by this blueprint.
                /// Used to detect dependency cycles due to mappers specifying each other as templates.
                bool buildAttempted;
            };


        private:
            // -------- INSTANCE VARIABLES --------------------------------- //

            /// Holds all known mapper blueprints.
            std::map<std::wstring_view, SBlueprint> blueprints;


        public:
            // -------- INSTANCE METHODS ----------------------------------- //

            /// Attempts to build mapper objects based on all of the blueprints known to this mapper builder object.
            /// Once a build attempt is made on a blueprint, that blueprint can no longer be modified.
            /// @return `true` if successful in building all of them, `false` otherwise.
            bool Build(void);

            /// Attempts to use a blueprint to build a mapper object of the specified name.
            /// Once a build attempt is made on a blueprint, that blueprint can no longer be modified.
            /// This method will fail if a mapper already exists with the specified name or if there is a blueprint template issue.
            /// If this method succeeds, then a mapper object was successfully created and can now be referenced by name.
            /// Any returned pointers are owned by the internal mapper registry.
            /// @param [in] mapperName Name that identifies the mapper described by a blueprint.
            /// @return Pointer to the new mapper object if successful, `nullptr` otherwise.
            const Mapper* Build(std::wstring_view mapperName);

            /// Removes an element mapper from this blueprint's element map specification so it is not applied as a modification to the template when this object is built into a mapper.
            /// This method will fail if the mapper name does not identify an existing blueprint, if the element index is out of bounds, or if no template modification exists for the specified controller element.
            /// @param [in] mapperName Name that identifies the mapper whose element is being set.
            /// @param [in] elementIndex Index of the element within the element map data structure (i.e. the `all` member of #UElementMap).
            /// @return `true` if successful, `false` otherwise.
            bool ClearBlueprintElementMapper(std::wstring_view mapperName, unsigned int elementIndex);

            /// Convenience wrapper for both parsing a controller element string and clearing an associated template modification.
            /// In addition to other reasons why this operation might fail, this method will fail if the element string cannot be mapped to a valid controller element.
            /// @param [in] mapperName Name that identifies the mapper whose element is being set.
            /// @param [in] elementString String that identifies the controller element. Must be null-terminated.
            /// @return `true` if successful, `false` otherwise.
            bool ClearBlueprintElementMapper(std::wstring_view mapperName, std::wstring_view elementString);

            /// Creates a new mapper blueprint object with the specified mapper name.
            /// This method will fail if a mapper or mapper blueprint already exists with the specified name.
            /// @param [in] mapperName Name that identifies the mapper to be described by the blueprint.
            /// @return `true` if successful, `false` otherwise.
            bool CreateBlueprint(std::wstring_view mapperName);

            /// Determines if the specified mapper name already exists as a blueprint within this object.
            /// @param [in] mapperName Name that identifies the mapper described by a possibly-existing blueprint.
            /// @return `true` if the mapper name already exists, `false` otherwise.
            bool DoesBlueprintNameExist(std::wstring_view mapperName) const;

            /// Retrieves and returns a read-only pointer to the element map specification associated with the blueprint for the mapper of the specified name.
            /// Primarily useful for testing.
            /// @param [in] mapperName Name that identifies the mapper described by a possibly-existing blueprint.
            /// @return Pointer to the blueprint's element map if the blueprint exists, or `nullptr` otherwise.
            const TElementMapSpec* GetBlueprintElementMapSpec(std::wstring_view mapperName) const;

            /// Retrieves and returns the template name associated with the blueprint for the mapper of the specified name.
            /// @param [in] mapperName Name that identifies the mapper described by a possibly-existing blueprint.
            /// @return Template name associated with the blueprint if the blueprint exists.
            std::optional<std::wstring_view> GetBlueprintTemplate(std::wstring_view mapperName) const;

            /// Sets a specific element mapper to be applied as a modification to the template when this object is built into a mapper.
            /// If `nullptr` is specified, then the modification to be applied to the template is element mapper removal. Use #ClearBlueprintElementMapper to undo a modification.
            /// This method will fail if the mapper name does not identify an existing blueprint or if the element index is out of bounds.
            /// @param [in] mapperName Name that identifies the mapper whose element is being set.
            /// @param [in] elementIndex Index of the element within the element map data structure (i.e. the `all` member of #UElementMap).
            /// @param [in] elementMapper Element mapper to use, which becomes owned by this object.
            /// @return `true` if successful, `false` otherwise.
            bool SetBlueprintElementMapper(std::wstring_view mapperName, unsigned int elementIndex, std::unique_ptr<IElementMapper>&& elementMapper);

            /// Convenience wrapper for both parsing a controller element string and applying it as a template modification.
            /// In addition to other reasons why this operation might fail, this method will fail if the element string cannot be mapped to a valid controller element.
            /// @param [in] mapperName Name that identifies the mapper whose element is being set.
            /// @param [in] elementString String that identifies the controller element. Must be null-terminated.
            /// @param [in] elementMapper Element mapper to use, which becomes owned by this object.
            /// @return `true` if successful, `false` otherwise.
            bool SetBlueprintElementMapper(std::wstring_view mapperName, std::wstring_view elementString, std::unique_ptr<IElementMapper>&& elementMapper);

            /// Sets the name of the mapper that will act as a template for the mapper being built.
            /// Template names are resolved when attempting to construct a mapper object, so it is not necessary for the template name to identify an existing mapper or mapper blueprint.
            /// This method will fail if the mapper name does not identify an existing blueprint.
            /// @param [in] mapperName Name that identifies the mapper whose template is being set.
            /// @param [in] newTemplateName New template name.
            /// @return `true` if successful, `false` otherwise.
            bool SetBlueprintTemplate(std::wstring_view mapperName, std::wstring_view newTemplateName);
        };
    }
}