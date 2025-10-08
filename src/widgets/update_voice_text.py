def update_voice_text(self, text):
    """Update the voice text label with recognized text."""
    if not text:
        return
        
    print(f"DEBUG: Updating voice text with: {text}")
    
    # Update the label
    self.voice_text_label.setText(f"Hearing: {text}")
    
    # Quick check for navigation intent
    navigation_triggers = ["go to", "take me to", "navigate to", "move to", "find"]
    if any(trigger in text.lower() for trigger in navigation_triggers):
        # Found potential navigation command, highlight it in the UI
        self.voice_text_label.setStyleSheet("""
            QLabel {
                background-color: #e3f2fd;
                padding: 8px;
                border-radius: 4px;
                border: 1px solid #2196F3;
                min-height: 20px;
                color: #0d47a1;
            }
        """)
        
        # We don't actually navigate yet - wait for button release
        # But we can indicate that a navigation command was detected
        self.voice_text_label.setText(f"Hearing navigation command: {text}")