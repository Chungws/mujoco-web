"""
ELO rating calculation module

Implements standard ELO rating algorithm with:
- K-factor = 32
- Initial rating = 1500
- Support for ties (score = 0.5)
- Support for both_bad votes (score = 0.25)
- Bradley-Terry confidence interval calculation
"""

import math


# ELO constants
INITIAL_ELO = 1500
K_FACTOR = 32


def calculate_elo(rating_a: float, rating_b: float, result: float, k: int = K_FACTOR) -> float:
    """
    Calculate new ELO rating for model A

    Args:
        rating_a: Current ELO rating of model A
        rating_b: Current ELO rating of model B
        result: Score for model A (0.0 to 1.0)
                - 1.0 = A wins
                - 0.5 = tie
                - 0.25 = both_bad
                - 0.0 = B wins
        k: K-factor (rating sensitivity, default: 32)

    Returns:
        float: New ELO rating for model A

    Example:
        >>> calculate_elo(1500, 1500, 1.0)  # A wins
        1516.0
        >>> calculate_elo(1500, 1500, 0.0)  # A loses
        1484.0
        >>> calculate_elo(1500, 1500, 0.5)  # Tie
        1500.0
    """
    # Expected score: E_A = 1 / (1 + 10^((R_B - R_A) / 400))
    expected_a = 1 / (1 + 10 ** ((rating_b - rating_a) / 400))

    # New rating: R'_A = R_A + K * (S_A - E_A)
    new_rating_a = rating_a + k * (result - expected_a)

    return new_rating_a


def get_score_from_vote(vote: str, is_left: bool) -> float:
    """
    Convert vote to ELO score for a specific model

    Args:
        vote: Vote result ("left_better", "right_better", "tie", "both_bad")
        is_left: True if calculating for left model, False for right model

    Returns:
        float: Score (S) for ELO calculation

    Raises:
        ValueError: If vote type is invalid

    Example:
        >>> get_score_from_vote("left_better", is_left=True)
        1.0
        >>> get_score_from_vote("left_better", is_left=False)
        0.0
        >>> get_score_from_vote("tie", is_left=True)
        0.5
        >>> get_score_from_vote("both_bad", is_left=True)
        0.25
    """
    if vote == "both_bad":
        # Small penalty for both models (LM Arena approach)
        return 0.25
    elif vote == "tie":
        return 0.5
    elif vote == "left_better":
        return 1.0 if is_left else 0.0
    elif vote == "right_better":
        return 0.0 if is_left else 1.0
    else:
        raise ValueError(f"Invalid vote type: {vote}")


def calculate_ci(vote_count: int) -> float:
    """
    Calculate 95% confidence interval using Bradley-Terry model

    Formula:
        SE = 400 / sqrt(n)  (Standard Error)
        CI = 1.96 * SE      (95% Confidence Interval)

    Args:
        vote_count: Number of votes received

    Returns:
        float: 95% confidence interval (rounded to 1 decimal place)

    Example:
        >>> calculate_ci(0)
        200.0
        >>> calculate_ci(100)
        78.4
        >>> calculate_ci(1000)
        24.8
    """
    if vote_count == 0:
        # Default CI for models with no votes
        return 200.0

    # Standard Error: SE = 400 / sqrt(n)
    se = 400 / math.sqrt(vote_count)

    # 95% Confidence Interval: CI = 1.96 * SE
    ci = 1.96 * se

    return round(ci, 1)
