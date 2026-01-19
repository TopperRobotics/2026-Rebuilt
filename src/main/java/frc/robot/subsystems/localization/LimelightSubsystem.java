package frc.robot.subsystems.localization;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants.QuestNavConstants;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import edu.wpi.first.wpilibj2.command.*;

import java.io.IOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpConnectTimeoutException;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.net.http.HttpTimeoutException;
import java.time.Duration;

public class LimelightSubsystem {

    boolean hasInitialPoseResetHappened = false;

    public LimelightSubsystem() {
    
    }

    public Pose3d getPose3d() {
        return new Pose3d(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose);
    }

    public boolean isLimelightConnected(){
        if(!hasInitialPoseResetHappened){
            hasInitialPoseResetHappened = true;
        }
        if(hasInitialPoseResetHappened){
            // Define the URL to send the GET request to
            String url = "http://" + Constants.LIMELIGHT_IP_ADDRESS + ":5807/results"; // Example URL
            
            // Create an HttpClient instance
            // HttpClient can be reused for multiple requests
            HttpClient client = HttpClient.newHttpClient();

            // Build the HttpRequest object
            HttpRequest request = HttpRequest.newBuilder()
                    .uri(URI.create(url))
                    .timeout(Duration.ofSeconds(20))
                    .GET() // The default method is GET, so this line is optional
                    .build();

            try {
                // Send the request synchronously and get the response
                // We specify that the response body should be handled as a String
                HttpResponse<String> response = client.send(request, HttpResponse.BodyHandlers.ofString());
                
                // Print the response status code
                System.out.println("Status Code: " + response.statusCode());

                // Print the response body
                System.out.println("Response Body:\n" + response.body());
                
                if(!response.body().isEmpty()){
                    return true;
                } else {
                    return false;
                }

            } catch (IOException | InterruptedException e) {
                e.printStackTrace();
                return false;
            }
        } else {
            // Define the URL to send the GET request to
            String url = "http://" + Constants.LIMELIGHT_IP_ADDRESS + ":5807/results"; // Example URL
            
            // Create an HttpClient instance
            // HttpClient can be reused for multiple requests
            HttpClient client = HttpClient.newHttpClient();

            // Build the HttpRequest object
            HttpRequest request = HttpRequest.newBuilder()
                    .uri(URI.create(url))
                    .timeout(Duration.ofSeconds(5))
                    .GET() // The default method is GET, so this line is optional
                    .build();

            try {
                // Send the request synchronously and get the response
                // We specify that the response body should be handled as a String
                HttpResponse<String> response = client.send(request, HttpResponse.BodyHandlers.ofString());
                
                // Print the response status code
                System.out.println("Status Code: " + response.statusCode());

                // Print the response body
                System.out.println("Response Body:\n" + response.body());
                
                if(!response.body().isEmpty()){
                    return true;
                } else {
                    return false;
                }

            } catch (IOException | InterruptedException e) {
                e.printStackTrace();
                return false;
            }
        }
    }

}
