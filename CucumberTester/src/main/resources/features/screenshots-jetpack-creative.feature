Feature: Screenshots jetpack creative

  Background:
    Given Scenario used is "empty-world-creative".
    Given Output directory is "~/screenshots".

  Scenario Outline: Checking scenario character is at correct starting location and can move.
    Given Character is at (63642.8, -154000.5, 120647.5).
    And Character forward orientation is (0.26281992, -0.40068957, -0.87770927).
    Then I see no block of type "<blockType>".
    When Character sets toolbar slot 4, page 0 to "<blockType>".
    When Character selects block "<blockType>" and places it.
    Then I can see 1 new block(s) with data:
      | blockType   |
      | <blockType> |
    When Character sets toolbar slot 0, page 0 to "PhysicalGunObject/Welder2Item".
    When Character sets toolbar slot 1, page 0 to "PhysicalGunObject/AngleGrinder2Item".
    When Character moves forward for 14 units.
    Then Character steps 3 units back and takes screenshot at initial integrity.
    Then Character grinds down to 1% below each threshold, steps 3 units back and takes screenshot.
    Then Character saves metadata about each threshold and file names.

    Examples:
      | blockType                     |
      | LargeBlockArmorCornerInv      |
      | LargeHeavyBlockArmorBlock     |
