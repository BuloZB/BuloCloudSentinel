@echo off

REM Set environment variables
set PYTHONPATH=%PYTHONPATH%;%CD%;%CD%\backend

REM Run the tests
python -m unittest discover -s backend/tests
