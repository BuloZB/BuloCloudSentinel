"""
Cesium 3D Tiles converter for the Mapping Service.
"""

import asyncio
import json
import logging
import os
import shutil
import tempfile
from typing import Any, Dict, List, Optional, Tuple

from mapping_service.core.config import settings

logger = logging.getLogger(__name__)


class CesiumConverter:
    """Converter for Cesium 3D Tiles."""
    
    def __init__(self):
        """Initialize the Cesium 3D Tiles converter."""
        self.temp_dir = settings.TEMP_DIRECTORY
        os.makedirs(self.temp_dir, exist_ok=True)
    
    async def convert_mesh_to_3d_tiles(self, mesh_path: str) -> Tuple[bool, Optional[str], Optional[str]]:
        """
        Convert a 3D mesh to Cesium 3D Tiles.
        
        Args:
            mesh_path: Path to the input mesh
            
        Returns:
            Tuple[bool, Optional[str], Optional[str]]: (success, output_path, error_message)
        """
        try:
            # Create temporary output directory
            output_dir = tempfile.mkdtemp(dir=self.temp_dir)
            
            # Extract mesh if it's a ZIP file
            if mesh_path.endswith(".zip"):
                extract_dir = tempfile.mkdtemp(dir=self.temp_dir)
                shutil.unpack_archive(mesh_path, extract_dir)
                
                # Find OBJ file
                obj_files = []
                for root, _, files in os.walk(extract_dir):
                    for file in files:
                        if file.endswith(".obj"):
                            obj_files.append(os.path.join(root, file))
                
                if not obj_files:
                    return False, None, "No OBJ file found in the mesh archive"
                
                mesh_path = obj_files[0]
            
            # Run 3D Tiles converter
            cmd = [
                "3d-tiles-converter",
                "--input", mesh_path,
                "--output", output_dir,
                "--format", "b3dm",
                "--tileset-options", json.dumps({
                    "geometricError": 500,
                    "refine": "ADD",
                    "maximumScreenSpaceError": 16
                })
            ]
            
            logger.info(f"Running 3D Tiles converter: {' '.join(cmd)}")
            process = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            
            stdout, stderr = await process.communicate()
            
            if process.returncode != 0:
                logger.error(f"3D Tiles converter failed: {stderr.decode()}")
                return False, None, f"3D Tiles converter failed: {stderr.decode()}"
            
            logger.info(f"3D Tiles converter completed successfully")
            
            # Check if tileset.json exists
            tileset_path = os.path.join(output_dir, "tileset.json")
            if not os.path.exists(tileset_path):
                return False, None, "Tileset.json not found in output directory"
            
            return True, output_dir, None
        except Exception as e:
            logger.error(f"Error converting mesh to 3D Tiles: {str(e)}")
            return False, None, f"Error converting mesh to 3D Tiles: {str(e)}"
    
    async def convert_orthomosaic_to_tiles(self, orthomosaic_path: str) -> Tuple[bool, Optional[str], Optional[str]]:
        """
        Convert an orthomosaic to map tiles.
        
        Args:
            orthomosaic_path: Path to the input orthomosaic
            
        Returns:
            Tuple[bool, Optional[str], Optional[str]]: (success, output_path, error_message)
        """
        try:
            # Create temporary output directory
            output_dir = tempfile.mkdtemp(dir=self.temp_dir)
            
            # Run GDAL to create tiles
            cmd = [
                "gdal2tiles.py",
                "--zoom=12-22",
                "--processes=4",
                "--webviewer=none",
                orthomosaic_path,
                output_dir
            ]
            
            logger.info(f"Running GDAL: {' '.join(cmd)}")
            process = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            
            stdout, stderr = await process.communicate()
            
            if process.returncode != 0:
                logger.error(f"GDAL failed: {stderr.decode()}")
                return False, None, f"GDAL failed: {stderr.decode()}"
            
            logger.info(f"GDAL completed successfully")
            
            # Check if tiles were created
            if not os.path.exists(os.path.join(output_dir, "tilemapresource.xml")):
                return False, None, "Tiles not found in output directory"
            
            return True, output_dir, None
        except Exception as e:
            logger.error(f"Error converting orthomosaic to tiles: {str(e)}")
            return False, None, f"Error converting orthomosaic to tiles: {str(e)}"
    
    async def create_cesium_ion_asset(self, asset_path: str, asset_type: str, name: str) -> Tuple[bool, Optional[str], Optional[str]]:
        """
        Create a Cesium Ion asset.
        
        Args:
            asset_path: Path to the asset
            asset_type: Asset type (3DTILES, IMAGERY, etc.)
            name: Asset name
            
        Returns:
            Tuple[bool, Optional[str], Optional[str]]: (success, asset_id, error_message)
        """
        try:
            # Check if Cesium Ion token is set
            if not settings.CESIUM_ION_TOKEN:
                return False, None, "Cesium Ion token not set"
            
            # Create temporary directory for upload
            upload_dir = tempfile.mkdtemp(dir=self.temp_dir)
            
            # Create metadata file
            metadata_path = os.path.join(upload_dir, "metadata.json")
            with open(metadata_path, "w") as f:
                json.dump({
                    "name": name,
                    "description": f"Generated by Bulo.Cloud Sentinel Mapping Service",
                    "type": asset_type,
                    "options": {
                        "sourceType": "3DTILES" if asset_type == "3DTILES" else "IMAGERY"
                    }
                }, f)
            
            # Create ZIP archive for upload
            zip_path = os.path.join(upload_dir, "asset.zip")
            if os.path.isdir(asset_path):
                shutil.make_archive(os.path.splitext(zip_path)[0], "zip", asset_path)
            else:
                # Create a ZIP with just the file
                import zipfile
                with zipfile.ZipFile(zip_path, "w") as zipf:
                    zipf.write(asset_path, os.path.basename(asset_path))
            
            # TODO: Implement Cesium Ion API client to upload asset
            # This is a placeholder for now
            
            return False, None, "Cesium Ion integration not implemented yet"
        except Exception as e:
            logger.error(f"Error creating Cesium Ion asset: {str(e)}")
            return False, None, f"Error creating Cesium Ion asset: {str(e)}"
