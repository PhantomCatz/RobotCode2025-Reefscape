// Copyright (c) 2025 FRC 2637
// https://github.com/PhantomCatz
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.CatzSubsystems.CatzDriveAndRobotOrientation.Drivetrain;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.wpilibj.CAN;
import java.util.ArrayList;
import java.util.List;

public class MT6835 {
  private CAN canDevice;
  private CANData canData;
  private final int deviceID;
  private final boolean debugMode;
  private long multiTurnCounts;

  // Default API ID for reading multi-turn counts
  private static final int DEFAULT_API_ID = 0; // Replace with your appropriate default API ID.
  private static final int QUERY_API_ID = 1; // API ID for querying devices

  public MT6835(int deviceID, boolean debugMode) {
    this.deviceID = deviceID;
    this.debugMode = debugMode;
    this.canDevice = new CAN(deviceID, 8, 10);
    this.canData = new CANData();
  }

  /**
   * Reads the latest CAN packet using the default API ID, parses 8 bytes into a 64-bit raw count,
   * and stores it in `multiTurnCounts`.
   *
   * <p>If no valid packet is received or data is < 8 bytes, `multiTurnCounts` is not updated.
   */
  public void readMultiTurnCounts() {
    // Use the default API ID for reading multi-turn counts
    int apiID = DEFAULT_API_ID;

    // Attempt to read the latest packet for apiID
    if (canDevice.readPacketLatest(apiID, canData)) {
      byte[] receivedData = canData.data;

      // We need at least 8 bytes to parse a 64-bit long
      if (canData.length >= 8) {
        // Combine the 8 bytes into one 64-bit long
        long rawCounts =
            ((long) (receivedData[0] & 0xFF) << 56)
                | ((long) (receivedData[1] & 0xFF) << 48)
                | ((long) (receivedData[2] & 0xFF) << 40)
                | ((long) (receivedData[3] & 0xFF) << 32)
                | ((long) (receivedData[4] & 0xFF) << 24)
                | ((long) (receivedData[5] & 0xFF) << 16)
                | ((long) (receivedData[6] & 0xFF) << 8)
                | ((long) (receivedData[7] & 0xFF));

        // Store it in our field
        multiTurnCounts = rawCounts;

        // Optional debug info
        if (debugMode) {
          System.out.printf(
              "Device %d: multiTurnCounts = %d (API_ID=0x%X, Data=%s)%n",
              deviceID, multiTurnCounts, apiID, bytesToHex(receivedData));
        }
      } else {
        // Not enough data to parse 64 bits
        if (debugMode) {
          System.out.printf(
              "Device %d: Received CAN packet with only %d bytes; need 8.%n",
              deviceID, canData.length);
        }
      }
    } else {
      // No CAN packet received
      if (debugMode) {
        System.out.printf("Device %d: No CAN message received for API_ID=0x%X.%n", deviceID, apiID);
      }
    }
  }

  /**
   * Returns the last stored raw multi-turn counts.
   *
   * @return The last known value of multiTurnCounts.
   */
  public long getMultiTurnCounts() {
    return multiTurnCounts;
  }

  /**
   * Sends a CAN packet to the device.
   *
   * @param apiID The API ID for the packet.
   * @param data The payload to send.
   */
  public void sendPacket(int apiID, byte[] data) {
    try {
      canDevice.writePacket(data, apiID);
      if (debugMode) {
        System.out.printf(
            "Device %d: Sent CAN Packet: API_ID=%d, Data=%s%n", deviceID, apiID, bytesToHex(data));
      }
    } catch (Exception e) {
      if (debugMode) {
        System.err.printf("Device %d: Failed to send CAN Packet: %s%n", deviceID, e.getMessage());
      }
    }
  }

  /**
   * Method to query devices using API ID 1. Continuously reads CAN packets with a timeout of 1000
   * ms until no new packets arrive. Stores all received data in an array and prints the received
   * devices.
   *
   * @return List of received CAN packet data as byte arrays.
   */
  public List<byte[]> queryDevices() {
    List<byte[]> receivedPackets = new ArrayList<>();
    boolean packetReceived;
    int responseCount = 0;

    do {
      packetReceived = canDevice.readPacketNew(QUERY_API_ID, canData);
      if (packetReceived) {
        responseCount++;
        byte[] dataCopy = new byte[canData.length];
        System.arraycopy(canData.data, 0, dataCopy, 0, canData.length);
        receivedPackets.add(dataCopy);
        System.out.printf(
            "Response %d: Received CAN Packet: Data=%s%n", responseCount, bytesToHex(dataCopy));
      }
    } while (packetReceived);

    System.out.printf("Total number of responses: %d%n", responseCount);

    if (receivedPackets.isEmpty()) {
      System.out.println("No CAN packets received during device query.");
    }

    return receivedPackets;
  }

  /**
   * Helper function to convert a byte array to a hexadecimal string.
   *
   * @param bytes The byte array to convert.
   * @return A string representation of the byte array in hexadecimal format.
   */
  private String bytesToHex(byte[] bytes) {
    StringBuilder sb = new StringBuilder();
    for (byte b : bytes) {
      sb.append(String.format("%02X ", b));
    }
    return sb.toString().trim();
  }
}
