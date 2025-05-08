import React, { useState, useEffect } from 'react';
import { 
  Box, 
  Button, 
  Card, 
  CardContent, 
  Container, 
  Grid, 
  Typography, 
  Tabs, 
  Tab, 
  Table, 
  TableBody, 
  TableCell, 
  TableContainer, 
  TableHead, 
  TableRow, 
  Paper, 
  Chip, 
  IconButton, 
  Dialog, 
  DialogTitle, 
  DialogContent, 
  DialogActions, 
  TextField, 
  FormControl, 
  InputLabel, 
  Select, 
  MenuItem,
  CircularProgress,
  Alert,
  Snackbar
} from '@mui/material';
import { 
  CloudUpload, 
  Delete, 
  Edit, 
  Refresh, 
  PlayArrow, 
  History, 
  CheckCircle, 
  Error as ErrorIcon,
  Info as InfoIcon
} from '@mui/icons-material';
import axios from 'axios';

function ModelHub() {
  const [tabValue, setTabValue] = useState(0);
  const [models, setModels] = useState([]);
  const [deployments, setDeployments] = useState([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);
  const [uploadDialogOpen, setUploadDialogOpen] = useState(false);
  const [deployDialogOpen, setDeployDialogOpen] = useState(false);
  const [selectedModel, setSelectedModel] = useState(null);
  const [uploadForm, setUploadForm] = useState({
    name: '',
    version: '',
    description: '',
    model_type: '',
    framework: '',
    stage: 'development',
    file: null
  });
  const [deployForm, setDeployForm] = useState({
    model_id: '',
    environment: 'production',
    deployment_type: 'blue-green',
    target: 'all',
    auto_rollback_enabled: true,
    rollback_threshold: 0.05
  });
  const [snackbar, setSnackbar] = useState({
    open: false,
    message: '',
    severity: 'info'
  });

  const handleTabChange = (event, newValue) => {
    setTabValue(newValue);
  };

  const fetchModels = async () => {
    try {
      setLoading(true);
      const response = await axios.get('/api/model-hub/models');
      setModels(response.data);
      setError(null);
    } catch (err) {
      console.error('Error fetching models:', err);
      setError('Failed to fetch models. Please try again later.');
    } finally {
      setLoading(false);
    }
  };

  const fetchDeployments = async () => {
    try {
      setLoading(true);
      const response = await axios.get('/api/model-hub/deployments');
      setDeployments(response.data);
      setError(null);
    } catch (err) {
      console.error('Error fetching deployments:', err);
      setError('Failed to fetch deployments. Please try again later.');
    } finally {
      setLoading(false);
    }
  };

  useEffect(() => {
    if (tabValue === 0) {
      fetchModels();
    } else {
      fetchDeployments();
    }
  }, [tabValue]);

  const handleUploadDialogOpen = () => {
    setUploadDialogOpen(true);
  };

  const handleUploadDialogClose = () => {
    setUploadDialogOpen(false);
    setUploadForm({
      name: '',
      version: '',
      description: '',
      model_type: '',
      framework: '',
      stage: 'development',
      file: null
    });
  };

  const handleDeployDialogOpen = (model) => {
    setSelectedModel(model);
    setDeployForm({
      ...deployForm,
      model_id: model.id
    });
    setDeployDialogOpen(true);
  };

  const handleDeployDialogClose = () => {
    setDeployDialogOpen(false);
    setSelectedModel(null);
    setDeployForm({
      model_id: '',
      environment: 'production',
      deployment_type: 'blue-green',
      target: 'all',
      auto_rollback_enabled: true,
      rollback_threshold: 0.05
    });
  };

  const handleUploadFormChange = (e) => {
    const { name, value } = e.target;
    setUploadForm({
      ...uploadForm,
      [name]: value
    });
  };

  const handleFileChange = (e) => {
    setUploadForm({
      ...uploadForm,
      file: e.target.files[0]
    });
  };

  const handleDeployFormChange = (e) => {
    const { name, value } = e.target;
    setDeployForm({
      ...deployForm,
      [name]: value
    });
  };

  const handleUploadSubmit = async (e) => {
    e.preventDefault();
    
    if (!uploadForm.file) {
      setSnackbar({
        open: true,
        message: 'Please select a file to upload',
        severity: 'error'
      });
      return;
    }
    
    const formData = new FormData();
    formData.append('file', uploadForm.file);
    formData.append('name', uploadForm.name);
    formData.append('version', uploadForm.version);
    formData.append('description', uploadForm.description);
    formData.append('model_type', uploadForm.model_type);
    formData.append('framework', uploadForm.framework);
    formData.append('stage', uploadForm.stage);
    
    try {
      setLoading(true);
      const response = await axios.post('/api/model-hub/models', formData, {
        headers: {
          'Content-Type': 'multipart/form-data'
        }
      });
      
      setSnackbar({
        open: true,
        message: 'Model uploaded successfully',
        severity: 'success'
      });
      
      handleUploadDialogClose();
      fetchModels();
    } catch (err) {
      console.error('Error uploading model:', err);
      setSnackbar({
        open: true,
        message: `Error uploading model: ${err.response?.data?.detail || err.message}`,
        severity: 'error'
      });
    } finally {
      setLoading(false);
    }
  };

  const handleDeploySubmit = async (e) => {
    e.preventDefault();
    
    try {
      setLoading(true);
      const response = await axios.post('/api/model-hub/deployments', deployForm);
      
      setSnackbar({
        open: true,
        message: 'Model deployment initiated successfully',
        severity: 'success'
      });
      
      handleDeployDialogClose();
      setTabValue(1); // Switch to deployments tab
    } catch (err) {
      console.error('Error deploying model:', err);
      setSnackbar({
        open: true,
        message: `Error deploying model: ${err.response?.data?.detail || err.message}`,
        severity: 'error'
      });
    } finally {
      setLoading(false);
    }
  };

  const handlePromoteDeployment = async (deploymentId) => {
    try {
      setLoading(true);
      const response = await axios.post(`/api/model-hub/deployments/${deploymentId}/promote`);
      
      setSnackbar({
        open: true,
        message: 'Deployment promoted successfully',
        severity: 'success'
      });
      
      fetchDeployments();
    } catch (err) {
      console.error('Error promoting deployment:', err);
      setSnackbar({
        open: true,
        message: `Error promoting deployment: ${err.response?.data?.detail || err.message}`,
        severity: 'error'
      });
    } finally {
      setLoading(false);
    }
  };

  const handleRollbackDeployment = async (deploymentId) => {
    try {
      setLoading(true);
      const response = await axios.post(`/api/model-hub/deployments/${deploymentId}/rollback`);
      
      setSnackbar({
        open: true,
        message: 'Deployment rolled back successfully',
        severity: 'success'
      });
      
      fetchDeployments();
    } catch (err) {
      console.error('Error rolling back deployment:', err);
      setSnackbar({
        open: true,
        message: `Error rolling back deployment: ${err.response?.data?.detail || err.message}`,
        severity: 'error'
      });
    } finally {
      setLoading(false);
    }
  };

  const handleCloseSnackbar = () => {
    setSnackbar({
      ...snackbar,
      open: false
    });
  };

  return (
    <Container maxWidth="lg" sx={{ mt: 4, mb: 4 }}>
      <Box sx={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', mb: 3 }}>
        <Typography variant="h4" component="h1">
          Model Hub
        </Typography>
        <Box>
          <Button 
            variant="contained" 
            color="primary" 
            startIcon={<CloudUpload />}
            onClick={handleUploadDialogOpen}
            sx={{ mr: 2 }}
          >
            Upload Model
          </Button>
          <Button 
            variant="outlined" 
            startIcon={<Refresh />}
            onClick={tabValue === 0 ? fetchModels : fetchDeployments}
          >
            Refresh
          </Button>
        </Box>
      </Box>

      <Tabs value={tabValue} onChange={handleTabChange} sx={{ mb: 3 }}>
        <Tab label="Models" />
        <Tab label="Deployments" />
      </Tabs>

      {error && (
        <Alert severity="error" sx={{ mb: 3 }}>
          {error}
        </Alert>
      )}

      {loading ? (
        <Box sx={{ display: 'flex', justifyContent: 'center', p: 3 }}>
          <CircularProgress />
        </Box>
      ) : (
        <>
          {tabValue === 0 && (
            <TableContainer component={Paper}>
              <Table>
                <TableHead>
                  <TableRow>
                    <TableCell>Name</TableCell>
                    <TableCell>Version</TableCell>
                    <TableCell>Type</TableCell>
                    <TableCell>Framework</TableCell>
                    <TableCell>Stage</TableCell>
                    <TableCell>Status</TableCell>
                    <TableCell>Actions</TableCell>
                  </TableRow>
                </TableHead>
                <TableBody>
                  {models.length === 0 ? (
                    <TableRow>
                      <TableCell colSpan={7} align="center">
                        No models found
                      </TableCell>
                    </TableRow>
                  ) : (
                    models.map((model) => (
                      <TableRow key={model.id}>
                        <TableCell>{model.name}</TableCell>
                        <TableCell>{model.version}</TableCell>
                        <TableCell>{model.model_type}</TableCell>
                        <TableCell>{model.framework}</TableCell>
                        <TableCell>
                          <Chip 
                            label={model.stage} 
                            color={
                              model.stage === 'production' ? 'success' :
                              model.stage === 'staging' ? 'warning' :
                              'default'
                            }
                            size="small"
                          />
                        </TableCell>
                        <TableCell>
                          {model.is_active ? (
                            <Chip label="Active" color="success" size="small" />
                          ) : (
                            <Chip label="Inactive" size="small" />
                          )}
                        </TableCell>
                        <TableCell>
                          <IconButton 
                            color="primary" 
                            onClick={() => handleDeployDialogOpen(model)}
                            title="Deploy"
                          >
                            <PlayArrow />
                          </IconButton>
                        </TableCell>
                      </TableRow>
                    ))
                  )}
                </TableBody>
              </Table>
            </TableContainer>
          )}

          {tabValue === 1 && (
            <TableContainer component={Paper}>
              <Table>
                <TableHead>
                  <TableRow>
                    <TableCell>Model</TableCell>
                    <TableCell>Environment</TableCell>
                    <TableCell>Type</TableCell>
                    <TableCell>Status</TableCell>
                    <TableCell>Metrics</TableCell>
                    <TableCell>Actions</TableCell>
                  </TableRow>
                </TableHead>
                <TableBody>
                  {deployments.length === 0 ? (
                    <TableRow>
                      <TableCell colSpan={6} align="center">
                        No deployments found
                      </TableCell>
                    </TableRow>
                  ) : (
                    deployments.map((deployment) => (
                      <TableRow key={deployment.id}>
                        <TableCell>{deployment.model_name || deployment.model_id}</TableCell>
                        <TableCell>
                          <Chip 
                            label={deployment.environment} 
                            color={deployment.environment === 'production' ? 'error' : 'warning'}
                            size="small"
                          />
                        </TableCell>
                        <TableCell>{deployment.deployment_type}</TableCell>
                        <TableCell>
                          <Chip 
                            label={deployment.status} 
                            color={
                              deployment.status === 'running' ? 'success' :
                              deployment.status === 'pending' ? 'warning' :
                              deployment.status === 'failed' ? 'error' :
                              'default'
                            }
                            size="small"
                          />
                        </TableCell>
                        <TableCell>
                          {deployment.fps && (
                            <Typography variant="body2">
                              FPS: {deployment.fps.toFixed(1)}
                            </Typography>
                          )}
                          {deployment.map && (
                            <Typography variant="body2">
                              mAP: {deployment.map.toFixed(3)}
                            </Typography>
                          )}
                        </TableCell>
                        <TableCell>
                          {deployment.status === 'running' && (
                            <>
                              <IconButton 
                                color="primary" 
                                onClick={() => handlePromoteDeployment(deployment.id)}
                                title="Promote"
                              >
                                <CheckCircle />
                              </IconButton>
                              <IconButton 
                                color="error" 
                                onClick={() => handleRollbackDeployment(deployment.id)}
                                title="Rollback"
                              >
                                <History />
                              </IconButton>
                            </>
                          )}
                        </TableCell>
                      </TableRow>
                    ))
                  )}
                </TableBody>
              </Table>
            </TableContainer>
          )}
        </>
      )}

      {/* Upload Dialog */}
      <Dialog open={uploadDialogOpen} onClose={handleUploadDialogClose} maxWidth="md" fullWidth>
        <DialogTitle>Upload Model</DialogTitle>
        <DialogContent>
          <Box component="form" onSubmit={handleUploadSubmit} sx={{ mt: 2 }}>
            <Grid container spacing={2}>
              <Grid item xs={6}>
                <TextField
                  name="name"
                  label="Model Name"
                  value={uploadForm.name}
                  onChange={handleUploadFormChange}
                  fullWidth
                  required
                  margin="normal"
                />
              </Grid>
              <Grid item xs={6}>
                <TextField
                  name="version"
                  label="Version"
                  value={uploadForm.version}
                  onChange={handleUploadFormChange}
                  fullWidth
                  required
                  margin="normal"
                />
              </Grid>
              <Grid item xs={12}>
                <TextField
                  name="description"
                  label="Description"
                  value={uploadForm.description}
                  onChange={handleUploadFormChange}
                  fullWidth
                  multiline
                  rows={2}
                  margin="normal"
                />
              </Grid>
              <Grid item xs={6}>
                <TextField
                  name="model_type"
                  label="Model Type"
                  value={uploadForm.model_type}
                  onChange={handleUploadFormChange}
                  fullWidth
                  required
                  margin="normal"
                  helperText="e.g., yolov10, sam, super-gradients"
                />
              </Grid>
              <Grid item xs={6}>
                <TextField
                  name="framework"
                  label="Framework"
                  value={uploadForm.framework}
                  onChange={handleUploadFormChange}
                  fullWidth
                  required
                  margin="normal"
                  helperText="e.g., pytorch, onnx, tflite"
                />
              </Grid>
              <Grid item xs={6}>
                <FormControl fullWidth margin="normal">
                  <InputLabel>Stage</InputLabel>
                  <Select
                    name="stage"
                    value={uploadForm.stage}
                    onChange={handleUploadFormChange}
                    label="Stage"
                  >
                    <MenuItem value="development">Development</MenuItem>
                    <MenuItem value="staging">Staging</MenuItem>
                    <MenuItem value="production">Production</MenuItem>
                  </Select>
                </FormControl>
              </Grid>
              <Grid item xs={6}>
                <Box sx={{ mt: 3 }}>
                  <Button
                    variant="contained"
                    component="label"
                    startIcon={<CloudUpload />}
                    fullWidth
                  >
                    Select Model File
                    <input
                      type="file"
                      hidden
                      onChange={handleFileChange}
                    />
                  </Button>
                  {uploadForm.file && (
                    <Typography variant="body2" sx={{ mt: 1 }}>
                      Selected file: {uploadForm.file.name}
                    </Typography>
                  )}
                </Box>
              </Grid>
            </Grid>
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={handleUploadDialogClose}>Cancel</Button>
          <Button 
            onClick={handleUploadSubmit} 
            variant="contained" 
            color="primary"
            disabled={loading}
          >
            {loading ? <CircularProgress size={24} /> : 'Upload'}
          </Button>
        </DialogActions>
      </Dialog>

      {/* Deploy Dialog */}
      <Dialog open={deployDialogOpen} onClose={handleDeployDialogClose} maxWidth="sm" fullWidth>
        <DialogTitle>Deploy Model</DialogTitle>
        <DialogContent>
          {selectedModel && (
            <Box sx={{ mb: 2 }}>
              <Typography variant="subtitle1">
                {selectedModel.name} (v{selectedModel.version})
              </Typography>
              <Typography variant="body2" color="text.secondary">
                {selectedModel.description}
              </Typography>
            </Box>
          )}
          <Box component="form" onSubmit={handleDeploySubmit} sx={{ mt: 2 }}>
            <Grid container spacing={2}>
              <Grid item xs={12}>
                <FormControl fullWidth margin="normal">
                  <InputLabel>Environment</InputLabel>
                  <Select
                    name="environment"
                    value={deployForm.environment}
                    onChange={handleDeployFormChange}
                    label="Environment"
                  >
                    <MenuItem value="production">Production</MenuItem>
                    <MenuItem value="staging">Staging</MenuItem>
                    <MenuItem value="development">Development</MenuItem>
                  </Select>
                </FormControl>
              </Grid>
              <Grid item xs={12}>
                <FormControl fullWidth margin="normal">
                  <InputLabel>Deployment Type</InputLabel>
                  <Select
                    name="deployment_type"
                    value={deployForm.deployment_type}
                    onChange={handleDeployFormChange}
                    label="Deployment Type"
                  >
                    <MenuItem value="blue-green">Blue-Green</MenuItem>
                    <MenuItem value="canary">Canary</MenuItem>
                    <MenuItem value="rolling">Rolling</MenuItem>
                  </Select>
                </FormControl>
              </Grid>
              <Grid item xs={12}>
                <FormControl fullWidth margin="normal">
                  <InputLabel>Target</InputLabel>
                  <Select
                    name="target"
                    value={deployForm.target}
                    onChange={handleDeployFormChange}
                    label="Target"
                  >
                    <MenuItem value="all">All Devices</MenuItem>
                    <MenuItem value="edge">Edge Devices Only</MenuItem>
                    <MenuItem value="cloud">Cloud Only</MenuItem>
                  </Select>
                </FormControl>
              </Grid>
            </Grid>
          </Box>
        </DialogContent>
        <DialogActions>
          <Button onClick={handleDeployDialogClose}>Cancel</Button>
          <Button 
            onClick={handleDeploySubmit} 
            variant="contained" 
            color="primary"
            disabled={loading}
          >
            {loading ? <CircularProgress size={24} /> : 'Deploy'}
          </Button>
        </DialogActions>
      </Dialog>

      <Snackbar 
        open={snackbar.open} 
        autoHideDuration={6000} 
        onClose={handleCloseSnackbar}
        anchorOrigin={{ vertical: 'bottom', horizontal: 'right' }}
      >
        <Alert onClose={handleCloseSnackbar} severity={snackbar.severity} sx={{ width: '100%' }}>
          {snackbar.message}
        </Alert>
      </Snackbar>
    </Container>
  );
}

export default ModelHub;
