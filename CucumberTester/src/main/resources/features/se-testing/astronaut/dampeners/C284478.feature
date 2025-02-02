Feature: C284478 Dampeners preserve their state when entering/exiting static or cockpit block that is not moving

  Scenario: C284478 Dampeners preserve their state when entering/exiting static or cockpit block that is not moving A
    Given Scenario config:
      | scenario           | main_client_medbay | observer_medbay |
      | character-movement | C284478-executor   | Observer Tier 1 |
    And Character dampeners are on.
    And UI dampeners are on.

    When Character uses.
    And Test waits for 1 seconds.
    And Character uses.
    Then Character dampeners are on.
    And UI dampeners are on.


  Scenario: C284478 Dampeners preserve their state when entering/exiting static or cockpit block that is not moving B
    # TODO: scenario with dampeners off
    Given Scenario config:
      | scenario           | main_client_medbay | observer_medbay |
      | character-movement | C284478-executor   | Observer Tier 1 |
    And Character dampeners are on.
    And UI dampeners are on.

    When Character turns off dampeners.
    And Character uses.
    Then Character dampeners are off.
    And UI dampeners are on.

    When Test waits for 1 seconds.
    And Character uses.
    Then dampeners are off.
    And Character dampeners are off.
    And UI dampeners are off.

