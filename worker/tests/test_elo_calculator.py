"""
Tests for ELO rating calculation

Tests ELO calculation algorithm:
- Standard ELO formula with K-factor = 32
- Initial rating = 1500
- Handle ties (score = 0.5)
- Handle both_bad (score = 0.25)
"""


class TestELOCalculation:
    """Test ELO rating calculation"""

    def test_calculate_elo_win(self):
        """Test ELO calculation when model A wins"""
        from vlaarena_worker.aggregators.elo_calculator import calculate_elo

        # Equal ratings (1500 vs 1500)
        # Expected score = 0.5, Actual score = 1.0
        # New rating = 1500 + 32 * (1.0 - 0.5) = 1516
        rating_a = 1500
        rating_b = 1500
        result = 1.0  # A wins

        new_rating = calculate_elo(rating_a, rating_b, result)
        assert new_rating == 1516.0

    def test_calculate_elo_loss(self):
        """Test ELO calculation when model A loses"""
        from vlaarena_worker.aggregators.elo_calculator import calculate_elo

        # Equal ratings (1500 vs 1500)
        # Expected score = 0.5, Actual score = 0.0
        # New rating = 1500 + 32 * (0.0 - 0.5) = 1484
        rating_a = 1500
        rating_b = 1500
        result = 0.0  # A loses

        new_rating = calculate_elo(rating_a, rating_b, result)
        assert new_rating == 1484.0

    def test_calculate_elo_tie(self):
        """Test ELO calculation for tie"""
        from vlaarena_worker.aggregators.elo_calculator import calculate_elo

        # Equal ratings (1500 vs 1500)
        # Expected score = 0.5, Actual score = 0.5
        # New rating = 1500 + 32 * (0.5 - 0.5) = 1500
        rating_a = 1500
        rating_b = 1500
        result = 0.5  # Tie

        new_rating = calculate_elo(rating_a, rating_b, result)
        assert new_rating == 1500.0

    def test_calculate_elo_both_bad(self):
        """Test ELO calculation for both_bad vote"""
        from vlaarena_worker.aggregators.elo_calculator import calculate_elo

        # Equal ratings (1500 vs 1500)
        # Expected score = 0.5, Actual score = 0.25
        # New rating = 1500 + 32 * (0.25 - 0.5) = 1492
        rating_a = 1500
        rating_b = 1500
        result = 0.25  # Both bad

        new_rating = calculate_elo(rating_a, rating_b, result)
        assert new_rating == 1492.0

    def test_calculate_elo_different_ratings(self):
        """Test ELO calculation with different ratings"""
        from vlaarena_worker.aggregators.elo_calculator import calculate_elo

        # Higher rated player wins (expected)
        # Rating A = 1600, Rating B = 1400
        # Expected score = 1 / (1 + 10^((1400-1600)/400)) = 0.76
        # New rating = 1600 + 32 * (1.0 - 0.76) = 1607.68
        rating_a = 1600
        rating_b = 1400
        result = 1.0  # A wins

        new_rating = calculate_elo(rating_a, rating_b, result)
        assert abs(new_rating - 1607.68) < 0.01

    def test_calculate_elo_upset(self):
        """Test ELO calculation when lower rated player wins (upset)"""
        from vlaarena_worker.aggregators.elo_calculator import calculate_elo

        # Lower rated player wins (upset)
        # Rating A = 1400, Rating B = 1600
        # Expected score = 1 / (1 + 10^((1600-1400)/400)) = 0.24
        # New rating = 1400 + 32 * (1.0 - 0.24) = 1424.32
        rating_a = 1400
        rating_b = 1600
        result = 1.0  # A wins (upset!)

        new_rating = calculate_elo(rating_a, rating_b, result)
        assert abs(new_rating - 1424.32) < 0.01


class TestVoteToScore:
    """Test vote string to ELO score conversion"""

    def test_get_score_left_better_left_model(self):
        """Test score when left model wins"""
        from vlaarena_worker.aggregators.elo_calculator import get_score_from_vote

        score = get_score_from_vote("left_better", is_left=True)
        assert score == 1.0

    def test_get_score_left_better_right_model(self):
        """Test score when left wins (from right model perspective)"""
        from vlaarena_worker.aggregators.elo_calculator import get_score_from_vote

        score = get_score_from_vote("left_better", is_left=False)
        assert score == 0.0

    def test_get_score_right_better_right_model(self):
        """Test score when right model wins"""
        from vlaarena_worker.aggregators.elo_calculator import get_score_from_vote

        score = get_score_from_vote("right_better", is_left=False)
        assert score == 1.0

    def test_get_score_right_better_left_model(self):
        """Test score when right wins (from left model perspective)"""
        from vlaarena_worker.aggregators.elo_calculator import get_score_from_vote

        score = get_score_from_vote("right_better", is_left=True)
        assert score == 0.0

    def test_get_score_tie(self):
        """Test score for tie vote"""
        from vlaarena_worker.aggregators.elo_calculator import get_score_from_vote

        # Tie should be 0.5 for both models
        score_left = get_score_from_vote("tie", is_left=True)
        score_right = get_score_from_vote("tie", is_left=False)
        assert score_left == 0.5
        assert score_right == 0.5

    def test_get_score_both_bad(self):
        """Test score for both_bad vote"""
        from vlaarena_worker.aggregators.elo_calculator import get_score_from_vote

        # Both bad should be 0.25 for both models
        score_left = get_score_from_vote("both_bad", is_left=True)
        score_right = get_score_from_vote("both_bad", is_left=False)
        assert score_left == 0.25
        assert score_right == 0.25


class TestConfidenceInterval:
    """Test confidence interval calculation using Bradley-Terry model"""

    def test_calculate_ci_zero_votes(self):
        """Test CI calculation with 0 votes (default)"""
        from vlaarena_worker.aggregators.elo_calculator import calculate_ci

        ci = calculate_ci(0)
        assert ci == 200.0

    def test_calculate_ci_one_vote(self):
        """Test CI calculation with 1 vote"""
        from vlaarena_worker.aggregators.elo_calculator import calculate_ci

        # SE = 400 / sqrt(1) = 400
        # CI = 1.96 * 400 = 784
        ci = calculate_ci(1)
        assert ci == 784.0

    def test_calculate_ci_hundred_votes(self):
        """Test CI calculation with 100 votes"""
        from vlaarena_worker.aggregators.elo_calculator import calculate_ci

        # SE = 400 / sqrt(100) = 40
        # CI = 1.96 * 40 = 78.4
        ci = calculate_ci(100)
        assert ci == 78.4

    def test_calculate_ci_thousand_votes(self):
        """Test CI calculation with 1000 votes"""
        from vlaarena_worker.aggregators.elo_calculator import calculate_ci

        # SE = 400 / sqrt(1000) = 12.649...
        # CI = 1.96 * 12.649 = 24.8
        ci = calculate_ci(1000)
        assert abs(ci - 24.8) < 0.1

    def test_calculate_ci_decreases_with_votes(self):
        """Test that CI decreases as vote count increases"""
        from vlaarena_worker.aggregators.elo_calculator import calculate_ci

        ci_10 = calculate_ci(10)
        ci_100 = calculate_ci(100)
        ci_1000 = calculate_ci(1000)

        # CI should decrease with more votes
        assert ci_10 > ci_100 > ci_1000
