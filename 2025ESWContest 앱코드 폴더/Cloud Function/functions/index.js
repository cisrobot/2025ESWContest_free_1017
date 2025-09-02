/**
 * Import function triggers from their respective submodules:
 *
 * const {onCall} = require("firebase-functions/v2/https");
 * const {onDocumentWritten} = require("firebase-functions/v2/firestore");
 *
 * See a full list of supported triggers at https://firebase.google.com/docs/functions
 */

const { onRequest } = require("firebase-functions/v2/https");
const { onDocumentWritten, onDocumentCreated } = require("firebase-functions/v2/firestore"); // Import v2 Firestore triggers
const { setGlobalOptions } = require("firebase-functions/v2"); // Import setGlobalOptions for region
const logger = require("firebase-functions/logger"); // v2 logger

const admin = require('firebase-admin');
admin.initializeApp();

// Set global options for all functions in this file.
// This is equivalent to .runWith({ region: 'asia-northeast3' }) for all functions by default,
// but can be overridden per function if needed.
setGlobalOptions({
    region: 'asia-northeast3'
});

/**
 * Cloud Function that sends a notification when the robot's state changes in Firestore.
 * It listens to 'Robot_State/current_state' document for write events.
 * This is now using onDocumentWritten from v2.
 */
exports.sendStateChangeNotification = onDocumentWritten('Robot_State/current_state', async (event) => {
    // For onDocumentWritten in v2, the change object is accessed via event.data
    const change = event.data;
    const newData = change.after.data();
    const oldData = change.before.data();

    if (!newData || !oldData || newData.state === oldData.state) {
        logger.log('No state change or invalid data, skipping notification');
        return null;
    }

    const state = newData.state;
    let title, body, topic;

    switch (state) {
        case '01':
            title = 'Patrol Complete';
            body = 'Robot patrol has completed.';
            topic = 'patrol_complete_en';
            break;
        case '11':
            title = 'Problem Detected';
            body = 'A problem has been detected with the robot.';
            topic = 'problem_detected_en';
            break;
        default:
            logger.log(`Unknown state: ${state}, using default topic`);
            title = 'Robot State Update';
            body = `Robot state changed to ${state}.`;
            topic = 'robot_state_en';
    }

    const payload = {
        data: {
            state: state,
            title: title,
            body: body
        },
        topic: topic
    };

    logger.log(`Sending notification to topic: ${topic}`, payload);

    try {
        const response = await admin.messaging().send(payload);
        logger.log(`Notification sent successfully: ${response}`);
        return null;
    } catch (error) {
        logger.error('Error sending notification:', error);
        // For background triggers, it's usually best to log the error and return,
        // or throw a standard Error if you want the function to be retried by Cloud Functions.
        throw new Error('Failed to send notification');
    }
});

/**
 * Cloud Function that sends a notification when new event GPS data is created in Firestore.
 * It listens to 'Event_Gps_Data/{eventId}' document for create events.
 * This is now using onDocumentCreated from v2.
 */
exports.sendEventNotification = onDocumentCreated('Event_Gps_Data/{eventId}', async (event) => {
    // For onDocumentCreated in v2, the snapshot is accessed via event.data
    const snap = event.data;
    const eventData = snap.data();
    const eventType = eventData.eventType || 'Unknown Event';
    const latitude = eventData.latitude || 0;
    const longitude = eventData.longitude || 0;
    // Ensure timestamp is a Date object before calling toLocaleString
    const timestamp = eventData.timestamp?.toDate ? eventData.timestamp.toDate().toLocaleString('ko-KR') : 'Unknown Time';
    const state = '10'; // 킥보드 감지 전용

    // Corrected Google Maps URL format
    const mapUrl = `https://www.google.com/maps/search/?api=1&query=$${latitude},${longitude}`;

    const title = `New Event: ${eventType}`;
    const body = `Latitude: ${latitude}, Longitude: ${longitude}\nLocation: ${mapUrl}`;

    const payload = {
        data: {
            state: state,
            title: title,
            body: body,
            eventType: eventType,
            latitude: latitude.toString(),
            longitude: longitude.toString(),
            timestamp: timestamp
        },
        topic: 'kickboard_detected_en'
    };

    logger.log(`Sending event notification to topic: kickboard_detected_en`, payload);

    try {
        const response = await admin.messaging().send(payload);
        logger.log(`Event notification sent successfully: ${response}`);
        return null;
    } catch (error) {
        logger.error('Error sending event notification:', error);
        throw new Error('Failed to send event notification');
    }
});
