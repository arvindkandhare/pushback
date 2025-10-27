// Testing Framework Demonstration
// This shows how to use the autonomous testing system

#include "main.h"
#include "autonomous_testing.h"

// Global testing instance
AutonomousTester* g_tester = nullptr;

void demo_testing_framework() {
    printf("🧪 AUTONOMOUS TESTING FRAMEWORK DEMO 🧪\n");
    
    // Initialize the testing system
    g_tester = new AutonomousTester();
    
    // Start a test
    g_tester->startTest("Demo Route Test");
    
    // Add some checkpoints
    g_tester->checkpoint("Initial setup complete");
    pros::delay(100);
    
    g_tester->checkpoint("Movement phase 1");
    pros::delay(200);
    
    g_tester->checkpoint("Scoring sequence");
    pros::delay(150);
    
    g_tester->checkpoint("Final positioning");
    pros::delay(100);
    
    // Complete the test
    g_tester->completeTest(15, true);  // 15 points, AWP achieved
    
    // Print the results
    printf("\n📊 TEST RESULTS:\n");
    g_tester->printResults();
    
    // Print success rate for this route
    double success_rate = g_tester->getSuccessRate("Demo Route Test");
    double avg_time = g_tester->getAverageTime("Demo Route Test");
    
    printf("\n⏱️ PERFORMANCE METRICS:\n");
    printf("Success Rate: %.1f%%\n", success_rate);
    printf("Average Time: %.2f seconds\n", avg_time);
    
    // Cleanup
    delete g_tester;
    g_tester = nullptr;
}

// Example showing comparison between routes
void demo_route_comparison() {
    printf("\n🏆 ROUTE COMPARISON DEMO 🏆\n");
    
    AutonomousTester tester;
    
    // Simulate multiple test runs for Red Right Bonus
    printf("Running simulated tests for Red Right Bonus...\n");
    tester.startTest("Red Right Bonus");
    tester.checkpoint("Phase 1 complete");
    tester.checkpoint("First scoring");
    tester.checkpoint("Match load");
    tester.completeTest(18, true);
    
    // Run another test for Red Right Bonus (with failure)
    tester.startTest("Red Right Bonus");
    tester.checkpoint("Phase 1 complete");
    tester.checkpoint("First scoring");
    tester.markFailure("Missed match load");
    tester.completeTest(12, false);
    
    // Simulate tests for Red Left Bonus
    printf("Running simulated tests for Red Left Bonus...\n");
    tester.startTest("Red Left Bonus");
    tester.checkpoint("Phase 1 complete");
    tester.checkpoint("First scoring");
    tester.checkpoint("Match load");
    tester.completeTest(15, false);
    
    tester.startTest("Red Left Bonus");
    tester.checkpoint("Phase 1 complete");
    tester.checkpoint("First scoring");
    tester.checkpoint("Match load");
    tester.completeTest(16, false);
    
    // Print comprehensive comparison
    printf("\nROUTE COMPARISON RESULTS:\n");
    tester.printRouteComparison();
    
    // Individual route analysis
    printf("\nINDIVIDUAL ROUTE ANALYSIS:\n");
    printf("Red Right Bonus:\n");
    printf("  Success Rate: %.1f%%\n", tester.getSuccessRate("Red Right Bonus"));
    printf("  Average Time: %.2fs\n", tester.getAverageTime("Red Right Bonus"));
    
    printf("Red Left Bonus:\n");
    printf("  Success Rate: %.1f%%\n", tester.getSuccessRate("Red Left Bonus"));
    printf("  Average Time: %.2fs\n", tester.getAverageTime("Red Left Bonus"));
}