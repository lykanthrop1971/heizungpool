async function updateSensorData() {
  try {
    const response = await fetch('/api/states');
    const states = await response.json();
    
    document.getElementById('vorlauftemperatur').textContent = 
      states.find(s => s.entity_id === 'sensor.vorlauftemperatur')?.state || 'N/A';
    document.getElementById('kesseltemperatur').textContent = 
      states.find(s => s.entity_id === 'sensor.kesseltemperatur')?.state || 'N/A';
    document.getElementById('warmwassertemperatur').textContent = 
      states.find(s => s.entity_id === 'sensor.warmwassertemperatur')?.state || 'N/A';
    document.getElementById('mischer_position').textContent = 
      states.find(s => s.entity_id === 'sensor.mischer_position')?.state || 'N/A';
    document.getElementById('heizkreispumpe').textContent = 
      states.find(s => s.entity_id === 'switch.heizkreispumpe')?.state || 'N/A';
    document.getElementById('warmwasserpumpe').textContent = 
      states.find(s => s.entity_id === 'switch.warmwasserpumpe')?.state || 'N/A';
    document.getElementById('brenner').textContent = 
      states.find(s => s.entity_id === 'switch.brenner')?.state || 'N/A';
  } catch (error) {
    console.error('Error fetching sensor data:', error);
  }
}

async function setHeizmodus() {
  const mode = document.getElementById('heizmodus').value;
  try {
    await fetch('/api/services/select/select_option', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        entity_id: 'select.heizmodus',
        option: mode
      })
    });
  } catch (error) {
    console.error('Error setting heizmodus:', error);
  }
}

async function setZieltemperatur() {
  const temp = document.getElementById('zieltemperatur').value;
  try {
    await fetch('/api/services/number/set_value', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        entity_id: 'number.zieltemperatur',
        value: parseFloat(temp)
      })
    });
  } catch (error) {
    console.error('Error setting zieltemperatur:', error);
  }
}

async function setWarmwasserZieltemperatur() {
  const temp = document.getElementById('warmwasser_zieltemperatur').value;
  try {
    await fetch('/api/services/number/set_value', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        entity_id: 'number.warmwasser_zieltemperatur',
        value: parseFloat(temp)
      })
    });
  } catch (error) {
    console.error('Error setting warmwasser zieltemperatur:', error);
  }
}

setInterval(updateSensorData, 5000);
window.onload = updateSensorData;