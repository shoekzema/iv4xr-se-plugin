Feature: C197574 Character can crouch / stand back up

  Background:
    Given Scenario used is "simple-place-grind-torch-with-tools-for-walking".

  Scenario: C197574 Character can crouch / stand back up
    Given Character is standing.
    When Character crouches.
    And Test waits for 1 seconds.
    Then Character is crouching.

# TODO: reverse scenario
