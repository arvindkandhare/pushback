#include "autonomous.h"
#include "lemlib_config.h"
#include "main.h"
#include <utility>
#include <cmath>  // For cos, sin functions

ASSET(RedRightBallCollection_txt);
ASSET(RedRightBallScore_txt);
ASSET(RedRightMoveToGoal_txt);
ASSET(RedRightMoveToMatchLoad_txt);

void stopIntakeAfterDelay() {
    chassis_turn->waitUntilDone();
    pros::delay(800);
    indexer_system->stopAll();
}

void midScoreAfterDelay() {
        chassis_turn->waitUntilDone();
        //pros::delay(400);
        indexer_system->setMidGoalMode();
        indexer_system->executeBack();
        
}

void AutonomousSystem::executeRedRightAWP() {

    printf("Executing Red Left AWP Route (Mirrored from proven Red Right route)\n");
    autonomous_running = true;

    // VERIFY PTO is in scorer mode (should already be set, but double-check)
    if (pto_system && !pto_system->isDrivetrainMode()) {
        printf("✅ Confirmed: PTO in scorer mode - middle wheels ready for scoring\n");
    } else {
        printf("⚠️  WARNING: PTO not in expected scorer mode - forcing scorer mode\n");
        pto_system->setScorerMode();
        pros::delay(200);
    }   

    chassis->setPose(-47, -12, 90);
    indexer_system->closeFrontFlap();
    indexer_system->startInput();
    chassis_turn->follow(RedRightBallCollection_txt, 8, 2500);
    
    pros::Task intakeTask(stopIntakeAfterDelay);
    //chassis_turn->waitUntilDone();
        //pros::delay(200);
    //indexer_system->stopAll();
    chassis_turn->turnToHeading(180, 1000, {.maxSpeed=120,.minSpeed=100, .earlyExitRange=10});
    
    
    
    chassis_turn->follow(RedRightBallScore_txt, 15, 2000, false);
    //pros::Task scoreTask(midScoreAfterDelay);

    chassis_turn->waitUntilDone();


    indexer_system->setMidGoalMode();
    indexer_system->executeBack();
    pros::delay(2500);


    //Deploy matchloader
    chassis_turn->follow(RedRightMoveToGoal_txt, 10, 10000, true);
    chassis_turn->waitUntilDone();
    
    chassis_turn->turnToHeading(-92, 2000, {.maxSpeed=80});
    indexer_system->stopAll();
    intake_system->deploy();
    pros::delay(2000);
    //indexer_system->startInput();
    indexer_system->startIntakeAndStorage();
    chassis_turn->follow(RedRightMoveToMatchLoad_txt, 8, 2000, true);
    //chassis->moveToPose(-65, chassis->getPose().y, chassis->getPose().theta, 5000,{.maxSpeed=100,.minSpeed=80});   
    chassis_turn->waitUntilDone();
    pros::delay(1500);
    chassis->moveToPose(-25, -51, -90, 5000,{.forwards=false,.minSpeed=120});
    
    indexer_system->setTopGoalMode();
    indexer_system->toggleStorageMode();
    indexer_system->executeBack();
    
    //Move into matchloader
    // brief pause for scoring

    //chassis_turn->setPose(-52, -6, 90);
    //Set intial position
    /*
    
    //Intake first 3 balls
    
    
    
    
    chassis->waitUntilDone();
    //Move to middle goal


    indexer_system->setMidGoalMode();
    indexer_system->executeBack();
    pros::delay(2500); // brief pause for scoring
    //pros::delay(3000); // brief pause for scoring
    //Score on middle goal


    indexer_system->stopAll();
    intake_system->deploy();
    //Deploy matchloader
    chassis->follow(RedRightMoveToGoal_txt, 8, 2000, true);
    chassis->waitUntilDone();
    //Move to match load


    

    chassis_turn->turnToHeading(270, 300, {.maxSpeed=120, .minSpeed=100, .earlyExitRange=3});
    indexer_system->startInput();
    chassis->moveToPose(-65, -47, 270, 5000,{.maxSpeed=120,.minSpeed=100});   
    //Move into matchloader

    pros::delay(3000); 



    


    //TODO
    //Get the straight pid different from the turn pid
    //Get the threading working
    //Get the intake methods working where I can spin the top intake slowly
    //Make it so that the pure pursuit can speed up and slow down
    //
    
    
    */
    
    /*
    // Set starting pose for LEFT side (mirror of Red Right's 60°)
    chassis->setPose(0, 0, 120);  // 120° = northwest direction (mirror of 60°)

    // START INTAKE
    indexer_system->startInput();

    // Move forward ~35.5" at mirrored angle (120° instead of 60°)
    chassis->moveToPoint(35.5 * sin(120 * M_PI / 180.0), 35.5 * cos(120 * M_PI / 180.0), 5000);
    chassis->waitUntilDone();
    
    pros::delay(100);
    
    // Turn to 180° (same as Red Right - facing toward red alliance)
    chassis->turnToHeading(180, 3000);
    chassis->waitUntilDone();
    
    pros::delay(100);

    // Back up ~12" (same positioning logic)
    auto pose = chassis->getPose();
    chassis->moveToPoint(pose.x - 12 * sin(180 * M_PI / 180.0), 
                       pose.y - 12 * cos(180 * M_PI / 180.0), 3000);
    chassis->waitUntilDone();

    // BACKSCORING MIDDLE - execute indexer back scoring sequence
    indexer_system->setMidGoalMode();
    indexer_system->executeBack();
    pros::delay(700); // brief pause for scoring
    indexer_system->stopAll();

    pros::delay(50);
    
    // Continue with mirrored navigation pattern
    pose = chassis->getPose();
    chassis->moveToPoint(pose.x + 27 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 27 * cos(pose.theta * M_PI / 180.0), 3000);
    chassis->waitUntilDone();
    
    // Mirror of 160° → 200° (opposite side approach)
    chassis->turnToHeading(200, 3000);
    chassis->waitUntilDone();
    
    pros::delay(50);
    
    pose = chassis->getPose();
    chassis->moveToPoint(pose.x + 22 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 22 * cos(pose.theta * M_PI / 180.0), 3000);
    chassis->waitUntilDone();
    
    pros::delay(50);
    
    // Mirror of 225° → 315° (approach match load from left side)
    chassis->turnToHeading(315, 3000);
    chassis->waitUntilDone();
    
    pose = chassis->getPose();
    chassis->moveToPoint(pose.x + 23.5 * sin(pose.theta * M_PI / 180.0),
                       pose.y + 23.5 * cos(pose.theta * M_PI / 180.0), 3000);
    chassis->waitUntilDone();

    pros::delay(1000);
    
    // START INTAKE FROM MATCH LOAD (left side)
    indexer_system->startInput();

    // Mirror of 231° → 309° (approach from left match load zone)
    chassis->turnToHeading(309, 3000);
    chassis->waitUntilDone();
    
    pose = chassis->getPose();
    chassis->moveToPoint(pose.x - 35 * sin(pose.theta * M_PI / 180.0),
                       pose.y - 35 * cos(pose.theta * M_PI / 180.0), 3000);
    chassis->waitUntilDone();

    pros::delay(50);

    // TOP BACKSCORING - use back/top indexer (same as Red Right)
    indexer_system->setTopGoalMode();
    indexer_system->executeBack();
    pros::delay(1200);
    indexer_system->stopAll();

    printf("Red Left AWP finished!\n");

    autonomous_running = false;
    printf("Red Left AWP Route Complete\n");

    */
}