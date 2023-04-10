import { RouteComponentProps, Router } from '@reach/router'

import { DisconnectionModal } from '@electricui/components-desktop-blueprint'
import { Header } from '../../components/Header'
import { Intent } from '@blueprintjs/core'

import React from 'react'

import {SetPointPage} from './Control_Panel'
import {PIDPage} from './PID_Panel'
import {CalPage} from './Cal_Panel'
import { navigate } from '@electricui/utility-electron'

interface InjectDeviceIDFromLocation {
  deviceID?: string
}

export const DevicePages = (
  props: RouteComponentProps & InjectDeviceIDFromLocation,
) => {
  if (!props.deviceID) {
    return <div>No deviceID?</div>
  }

  return (
    <React.Fragment>
      <DisconnectionModal
        intent={Intent.WARNING}
        icon="satellite"
        navigateToConnectionsScreen={() => navigate('/')}
      >
        <p>
          Connection has been lost with your device. If we successfully
          reconnect this dialog will be dismissed.
        </p>
      </DisconnectionModal>

      <div className="device-pages">
        <Header deviceID={props.deviceID} {...props} />
        <div className="device-content">
          <Router primary={false}>
            
            <SetPointPage path="Control_Panel" />
            <PIDPage path="PID_Panel" />
            <CalPage path="Cal_Panel" />

          </Router>
        </div>
      </div>
    </React.Fragment>
  )
}
