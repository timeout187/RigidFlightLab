# RigidFlightLab - academic 6-DOF spin-stabilized projectile simulator.
# Published-benchmark reproduction and numerical-methods education only.
# Not for operational use. See docs/safety.md.
FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

EXPOSE 8501

ENTRYPOINT ["streamlit", "run", "src/gui/app.py", "--server.port=8501", "--server.address=0.0.0.0"]
