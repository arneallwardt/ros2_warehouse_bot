import cv2

# Öffne die Kamera (Standardkamera ist normalerweise Index 0)
cap = cv2.VideoCapture(0)

# Überprüfe, ob die Kamera geöffnet werden konnte
if not cap.isOpened():
    print("Kamera konnte nicht geöffnet werden")
    exit()

while True:
    # Lese ein Frame von der Kamera
    ret, frame = cap.read()
    
    if not ret:
        print("Fehler beim Lesen des Frames")
        break
    
    # Zeige das aktuelle Frame an
    cv2.imshow("Kamera", frame)
    
    # Beende die Schleife, wenn die 'q'-Taste gedrückt wird
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Schließe die Kamera und alle Fenster
cap.release()
cv2.destroyAllWindows()
