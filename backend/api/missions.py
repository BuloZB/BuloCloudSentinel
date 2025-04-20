from typing import List
from fastapi import APIRouter, Depends, HTTPException, UploadFile, File
from sqlalchemy.ext.asyncio import AsyncSession
from pydantic import BaseModel, Field
from backend.infrastructure.database import get_session
from backend.infrastructure.persistence.models import MissionModel, WaypointModel
from sqlalchemy.future import select
from sqlalchemy import delete
import xml.etree.ElementTree as ET

router = APIRouter(prefix="/missions", tags=["Missions"])

# Pydantic schemas
class WaypointCreate(BaseModel):
    latitude: float
    longitude: float
    altitude: float = None
    action: str = None
    order: int

class MissionCreate(BaseModel):
    name: str = Field(..., max_length=100)
    description: str = None
    waypoints: List[WaypointCreate] = []

class WaypointOut(WaypointCreate):
    id: int

class MissionOut(BaseModel):
    id: int
    name: str
    description: str = None
    waypoints: List[WaypointOut] = []

# CRUD endpoints
@router.get("/", response_model=List[MissionOut])
async def list_missions(db: AsyncSession = Depends(get_session)):
    result = await db.execute(select(MissionModel).order_by(MissionModel.id))
    missions = result.scalars().unique().all()
    out = []
    for m in missions:
        waypoints = [
            WaypointOut(
                id=wp.id,
                latitude=wp.latitude,
                longitude=wp.longitude,
                altitude=wp.altitude,
                action=wp.action,
                order=wp.order
            ) for wp in sorted(m.waypoints, key=lambda w: w.order)
        ]
        out.append(MissionOut(id=m.id, name=m.name, description=m.description, waypoints=waypoints))
    return out

@router.post("/", response_model=MissionOut, status_code=201)
async def create_mission(payload: MissionCreate, db: AsyncSession = Depends(get_session)):
    mission = MissionModel(name=payload.name, description=payload.description)
    db.add(mission)
    await db.flush()  # assign ID
    for wp in payload.waypoints:
        waypoint = WaypointModel(
            mission_id=mission.id,
            latitude=wp.latitude,
            longitude=wp.longitude,
            altitude=wp.altitude,
            action=wp.action,
            order=wp.order
        )
        db.add(waypoint)
    await db.commit()
    return await list_missions(db)

@router.get("/{mission_id}", response_model=MissionOut)
async def get_mission(mission_id: int, db: AsyncSession = Depends(get_session)):
    mission = await db.get(MissionModel, mission_id)
    if not mission:
        raise HTTPException(404, "Mission not found")
    waypoints = [
        WaypointOut(
            id=wp.id,
            latitude=wp.latitude,
            longitude=wp.longitude,
            altitude=wp.altitude,
            action=wp.action,
            order=wp.order
        ) for wp in sorted(mission.waypoints, key=lambda w: w.order)
    ]
    return MissionOut(id=mission.id, name=mission.name, description=mission.description, waypoints=waypoints)

@router.put("/{mission_id}", response_model=MissionOut)
async def update_mission(mission_id: int, payload: MissionCreate, db: AsyncSession = Depends(get_session)):
    mission = await db.get(MissionModel, mission_id)
    if not mission:
        raise HTTPException(404, "Mission not found")
    mission.name = payload.name
    mission.description = payload.description
    # delete existing waypoints
    await db.execute(delete(WaypointModel).where(WaypointModel.mission_id == mission_id))
    await db.flush()
    # add new waypoints
    for wp in payload.waypoints:
        waypoint = WaypointModel(
            mission_id=mission.id,
            latitude=wp.latitude,
            longitude=wp.longitude,
            altitude=wp.altitude,
            action=wp.action,
            order=wp.order
        )
        db.add(waypoint)
    await db.commit()
    return await get_mission(mission_id, db)

@router.delete("/{mission_id}", status_code=204)
async def delete_mission(mission_id: int, db: AsyncSession = Depends(get_session)):
    mission = await db.get(MissionModel, mission_id)
    if not mission:
        raise HTTPException(404, "Mission not found")
    await db.delete(mission)
    await db.commit()
    return

@router.post("/{mission_id}/import")
async def import_mission(mission_id: int, file: UploadFile = File(...), db: AsyncSession = Depends(get_session)):
    content = await file.read()
    try:
        root = ET.fromstring(content)
        ns = {'kml': 'http://www.opengis.net/kml/2.2'}
        mission = await db.get(MissionModel, mission_id)
        if not mission:
            raise HTTPException(404, "Mission not found")
        # Clear existing waypoints
        await db.execute(delete(WaypointModel).where(WaypointModel.mission_id == mission_id))
        await db.flush()
        # Parse KML for coordinates and create waypoints
        for placemark in root.findall('.//kml:Placemark', ns):
            point = placemark.find('.//kml:Point/kml:coordinates', ns)
            if point is not None:
                coords_text = point.text.strip()
                lon, lat, *alt = map(float, coords_text.split(','))
                waypoint = WaypointModel(
                    mission_id=mission_id,
                    latitude=lat,
                    longitude=lon,
                    altitude=alt[0] if alt else None,
                    action=None,
                    order=0  # TODO: determine order properly
                )
                db.add(waypoint)
        await db.commit()
        return {"detail": "Import successful"}
    except Exception as e:
        raise HTTPException(400, f"Failed to import mission: {str(e)}")

@router.get("/{mission_id}/export")
async def export_mission(mission_id: int, db: AsyncSession = Depends(get_session)):
    mission = await db.get(MissionModel, mission_id)
    if not mission:
        raise HTTPException(404, "Mission not found")
    # Generate simple KML export
    kml_header = '''<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
  <name>{}</name>
'''.format(mission.name)
    kml_footer = '''
</Document>
</kml>'''
    placemarks = ''
    for wp in sorted(mission.waypoints, key=lambda w: w.order):
        placemarks += f'''
  <Placemark>
    <name>Waypoint {wp.order}</name>
    <Point>
      <coordinates>{wp.longitude},{wp.latitude},{wp.altitude or 0}</coordinates>
    </Point>
  </Placemark>'''
    kml_content = kml_header + placemarks + kml_footer
    return Response(content=kml_content, media_type="application/vnd.google-earth.kml+xml")
