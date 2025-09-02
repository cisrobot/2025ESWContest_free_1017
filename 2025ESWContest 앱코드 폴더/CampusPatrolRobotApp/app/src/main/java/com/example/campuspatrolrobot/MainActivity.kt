package com.example.campuspatrolrobot

import android.content.Intent
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.animation.animateContentSize
import androidx.compose.animation.core.spring
import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ChatBubble
import androidx.compose.material.icons.filled.Notifications
import androidx.compose.material.icons.filled.Place
import androidx.compose.material3.Icon
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.scale
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.unit.dp
import com.example.campuspatrolrobot.ui.theme.CampusPatrolRobotAppTheme
import com.google.firebase.ktx.Firebase
import com.google.firebase.messaging.ktx.messaging
import com.google.firebase.remoteconfig.ktx.remoteConfig
import com.google.firebase.remoteconfig.ktx.remoteConfigSettings
import androidx.compose.foundation.Image
import androidx.compose.material.icons.automirrored.filled.ArrowForward
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.graphics.Color
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.delay

class MainActivity : ComponentActivity() {
    // Initializes activity and sets up Firebase subscriptions
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        CoroutineScope(Dispatchers.IO).launch {
            Firebase.messaging.subscribeToTopic("patrol_complete_en")
            Firebase.messaging.subscribeToTopic("kickboard_detected_en")
            Firebase.messaging.subscribeToTopic("problem_detected_en")
        }
        val remoteConfig = Firebase.remoteConfig
        val configSettings = remoteConfigSettings {
            minimumFetchIntervalInSeconds = 0
        }
        remoteConfig.setConfigSettingsAsync(configSettings)
        remoteConfig.setDefaultsAsync(mapOf("auth_key" to "default_auth_key"))
        remoteConfig.fetchAndActivate()
        setContent {
            CampusPatrolRobotAppTheme(darkTheme = true) {
                Scaffold(
                    modifier = Modifier.fillMaxSize(),
                    containerColor = MaterialTheme.colorScheme.background
                ) { innerPadding ->
                    Column(
                        modifier = Modifier
                            .fillMaxSize()
                            .padding(innerPadding),
                        horizontalAlignment = Alignment.CenterHorizontally,
                        verticalArrangement = Arrangement.Center
                    ) {
                        Row(
                            modifier = Modifier.padding(bottom = 8.dp),
                            verticalAlignment = Alignment.CenterVertically
                        ) {
                            Image(
                                painter = painterResource(id = R.drawable.logo),
                                contentDescription = "Campus Patrol Robot Logo",
                                modifier = Modifier
                                    .size(48.dp)
                                    .padding(end = 8.dp)
                            )
                            Text(
                                text = "Campus Patrol Robot",
                                style = MaterialTheme.typography.headlineMedium,
                                color = MaterialTheme.colorScheme.onBackground
                            )
                        }
                        Column(
                            modifier = Modifier
                                .fillMaxWidth(0.8f)
                                .padding(bottom = 32.dp),
                            horizontalAlignment = Alignment.CenterHorizontally
                        ) {
                            Surface(
                                modifier = Modifier
                                    .fillMaxWidth()
                                    .height(1.dp),
                                color = Color.Gray.copy(alpha = 0.5f)
                            ) {}
                        }
                        ActionCard(
                            icon = Icons.Default.ChatBubble,
                            label = "Command Chat",
                            onClick = {
                                startActivity(Intent(this@MainActivity, ChatActivity1::class.java))
                            }
                        )
                        ActionCard(
                            icon = Icons.Default.Notifications,
                            label = "Event notification",
                            onClick = {
                                startActivity(Intent(this@MainActivity, ChatActivity2::class.java))
                            }
                        )
                        ActionCard(
                            icon = Icons.Default.Place,
                            label = "Real-time map",
                            onClick = {
                                startActivity(Intent(this@MainActivity, MapActivity::class.java))
                            }
                        )
                    }
                }
            }
        }
    }
}

// Renders interactive card for navigation actions
@Composable
fun ActionCard(icon: ImageVector, label: String, onClick: () -> Unit) {
    var isPressed by remember { mutableStateOf(false) }
    val animatedScale by animateFloatAsState(
        targetValue = if (isPressed) 0.95f else 1f,
        animationSpec = spring(),
        label = "scaleAnimation"
    )
    Surface(
        modifier = Modifier
            .fillMaxWidth(0.8f)
            .padding(vertical = 8.dp)
            .scale(animatedScale)
            .clickable(
                onClick = {
                    isPressed = true
                    CoroutineScope(Dispatchers.Main).launch {
                        delay(100)
                        onClick()
                        isPressed = false
                    }
                }
            )
            .animateContentSize(animationSpec = spring()),
        shape = RoundedCornerShape(16.dp),
        color = MaterialTheme.colorScheme.surface,
        tonalElevation = 4.dp
    ) {
        Row(
            modifier = Modifier
                .fillMaxWidth()
                .padding(16.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.SpaceBetween
        ) {
            Row(
                verticalAlignment = Alignment.CenterVertically
            ) {
                Icon(
                    imageVector = icon,
                    contentDescription = label,
                    tint = MaterialTheme.colorScheme.primary,
                    modifier = Modifier.padding(end = 16.dp)
                )
                Text(
                    text = label,
                    style = MaterialTheme.typography.bodyLarge,
                    color = MaterialTheme.colorScheme.onSurface
                )
            }
            Icon(
                imageVector = Icons.AutoMirrored.Filled.ArrowForward,
                contentDescription = "Navigate",
                tint = MaterialTheme.colorScheme.primary
            )
        }
    }
}