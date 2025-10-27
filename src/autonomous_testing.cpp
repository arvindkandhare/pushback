/**
 * \file autonomous_testing.cpp
 * 
 * Implementation of autonomous testing framework.
 */

#include "autonomous_testing.h"
#include <algorithm>
#include <cstring>

// Global tester instance
AutonomousTester autonomous_tester;

AutonomousTester::AutonomousTester() 
    : current_test_start_time(0), current_test(nullptr) {}

void AutonomousTester::startTest(const std::string& route_name) {
    test_results.emplace_back(route_name);
    current_test = &test_results.back();
    current_test_start_time = pros::millis();
    
    printf("≡ƒº¬ TESTING: %s - Started at %.2fs\n", 
           route_name.c_str(), current_test_start_time / 1000.0);
}

void AutonomousTester::checkpoint(const std::string& checkpoint_name) {
    if (!current_test) return;
    
    uint32_t elapsed = pros::millis() - current_test_start_time;
    printf("Γ£à CHECKPOINT: %s at %.2fs\n", 
           checkpoint_name.c_str(), elapsed / 1000.0);
}

void AutonomousTester::markFailure(const std::string& failure_reason) {
    if (!current_test) return;
    
    current_test->completed_successfully = false;
    current_test->failure_points.push_back(failure_reason);
    
    uint32_t elapsed = pros::millis() - current_test_start_time;
    printf("Γ¥î FAILURE: %s at %.2fs\n", 
           failure_reason.c_str(), elapsed / 1000.0);
}

void AutonomousTester::completeTest(int points_scored, bool awp_status) {
    if (!current_test) return;
    
    current_test->execution_time_ms = pros::millis() - current_test_start_time;
    current_test->estimated_points = points_scored;
    current_test->awp_completed = awp_status;
    
    if (current_test->failure_points.empty()) {
        current_test->completed_successfully = true;
    }
    
    printf("≡ƒÅü TEST COMPLETE: %s\n", current_test->route_name.c_str());
    printf("   Time: %.2fs | Points: %d | AWP: %s | Success: %s\n",
           current_test->execution_time_ms / 1000.0,
           points_scored,
           awp_status ? "Γ£à" : "Γ¥î",
           current_test->completed_successfully ? "Γ£à" : "Γ¥î");
    
    current_test = nullptr;
}

void AutonomousTester::printResults() {
    printf("\n≡ƒôè ==================== AUTONOMOUS TEST RESULTS ====================\n");
    
    for (const auto& result : test_results) {
        printf("\n≡ƒñû Route: %s\n", result.route_name.c_str());
        printf("   ΓÅ▒∩╕Å  Time: %.2fs\n", result.execution_time_ms / 1000.0);
        printf("   ≡ƒÆ░ Points: %d\n", result.estimated_points);
        printf("   ≡ƒÅå AWP: %s\n", result.awp_completed ? "COMPLETED" : "FAILED");
        printf("   Γ£à Success: %s\n", result.completed_successfully ? "YES" : "NO");
        
        if (!result.failure_points.empty()) {
            printf("   Γ¥î Failures:\n");
            for (const auto& failure : result.failure_points) {
                printf("      - %s\n", failure.c_str());
            }
        }
        printf("   ================================================\n");
    }
}

void AutonomousTester::printRouteComparison() {
    if (test_results.empty()) {
        printf("No test results available for comparison.\n");
        return;
    }
    
    printf("\n≡ƒôê ==================== ROUTE COMPARISON ====================\n");
    printf("%-20s | %-8s | %-8s | %-8s | %-8s\n", 
           "Route", "Success%", "Avg Time", "Avg Pts", "AWP Rate");
    printf("================================================================\n");
    
    // Group results by route name
    std::vector<std::string> unique_routes;
    for (const auto& result : test_results) {
        bool found = false;
        for (const auto& route : unique_routes) {
            if (route == result.route_name) {
                found = true;
                break;
            }
        }
        if (!found) {
            unique_routes.push_back(result.route_name);
        }
    }
    
    for (const auto& route : unique_routes) {
        double success_rate = getSuccessRate(route);
        double avg_time = getAverageTime(route);
        
        // Calculate averages
        int total_tests = 0;
        int total_points = 0;
        int awp_successes = 0;
        
        for (const auto& result : test_results) {
            if (result.route_name == route) {
                total_tests++;
                total_points += result.estimated_points;
                if (result.awp_completed) awp_successes++;
            }
        }
        
        double avg_points = total_tests > 0 ? (double)total_points / total_tests : 0;
        double awp_rate = total_tests > 0 ? (double)awp_successes / total_tests * 100 : 0;
        
        printf("%-20s | %-7.1f%% | %-7.2fs | %-7.1f | %-7.1f%%\n",
               route.c_str(), success_rate, avg_time, avg_points, awp_rate);
    }
    printf("================================================================\n");
}

double AutonomousTester::getSuccessRate(const std::string& route_name) {
    int total = 0;
    int successes = 0;
    
    for (const auto& result : test_results) {
        if (result.route_name == route_name) {
            total++;
            if (result.completed_successfully) successes++;
        }
    }
    
    return total > 0 ? (double)successes / total * 100 : 0;
}

double AutonomousTester::getAverageTime(const std::string& route_name) {
    int total = 0;
    uint32_t sum_time = 0;
    
    for (const auto& result : test_results) {
        if (result.route_name == route_name) {
            total++;
            sum_time += result.execution_time_ms;
        }
    }
    
    return total > 0 ? (double)sum_time / total / 1000.0 : 0;
}

void AutonomousTester::clearResults() {
    test_results.clear();
    current_test = nullptr;
    printf("≡ƒùæ∩╕Å Test results cleared.\n");
}
