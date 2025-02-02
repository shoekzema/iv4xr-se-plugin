Feature: Screenshots survival

  Background:
    Given Scenario used is "violent-survival".
    Given Output directory is "~/screenshots".

  Scenario Outline: Checking scenario character is at correct starting location and can move.
    Given Character is at (83813.0, -110846.6, 138588.4).
    And Character forward orientation is (-1, 0, 0).
    And I see no block of type "<blockType>".
    When Character sets toolbar slot 0, page 0 to "<blockType>".
    When Character sets toolbar slot 1, page 0 to "<blockType>".
    When Character sets toolbar slot 2, page 0 to "<blockType>".
    When Character sets toolbar slot 3, page 0 to "<blockType>".
    When Character selects block "<blockType>" and places it.
    Then I can see 1 new block(s) with data:
      | blockType   |
      | <blockType> |
    When Character sets toolbar slot 4, page 0 to "PhysicalGunObject/Welder2Item".
    When Character sets toolbar slot 5, page 0 to "PhysicalGunObject/Welder2Item".
    When Character sets toolbar slot 6, page 0 to "PhysicalGunObject/AngleGrinder2Item".
    When Character sets toolbar slot 7, page 0 to "PhysicalGunObject/AngleGrinder2Item".
    When Character moves forward for 4 units.
    Then Character steps 3 units back and takes screenshot at initial integrity.
    Then Character welds up to 1% above each threshold, steps 3 units back and takes screenshot.
    Then Character saves metadata about each threshold and file names.

    Examples:
      | blockType                     |
      | LargeBlockArmorCornerInv      |
      | LargeHeavyBlockArmorBlock     |
