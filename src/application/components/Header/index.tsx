import { Alignment, Button, Navbar } from '@blueprintjs/core'
import {
  useDeadline,
  useDeviceConnect,
  useDeviceConnectionRequested,
  useDeviceDisconnect,
} from '@electricui/components-core'

import React from 'react'
import { RouteComponentProps } from '@reach/router'
import { navigate } from '@electricui/utility-electron'

interface InjectDeviceIDFromLocation {
  deviceID?: string
  '*'?: string // we get passed the path as the wildcard
}

export const Header = (
  props: RouteComponentProps & InjectDeviceIDFromLocation,
) => {
  const disconnect = useDeviceDisconnect()
  const connect = useDeviceConnect()
  const connectionRequested = useDeviceConnectionRequested()
  const getDeadline = useDeadline()

  const page = props['*'] // we get passed the path as the wildcard, so we read it here

  return (
    <div className="device-header">
      <Navbar style={{ background: 'transparent', boxShadow: 'none' }}>
        <div style={{ margin: '0 auto', width: '100%' }}>
          <Navbar.Group align={Alignment.LEFT}>
            <Button
              minimal
              large
              icon="home"
              text="Back"
              onClick={() => {
                navigate('/')
              }}
            />

            {connectionRequested ? (
              <Button
                minimal
                intent="danger"
                icon="cross"
                text="Disconnect"
                onClick={() => {
                  disconnect().catch(err => {
                    console.warn('Failed to disconnect', err)
                  })
                }}
              />
            ) : (
              <Button
                minimal
                icon="link"
                intent="success"
                text="Connect again"
                onClick={() => {
                  const cancellationToken = getDeadline()

                  connect(cancellationToken).catch(err => {
                    if (cancellationToken.caused(err)) {
                      return
                    }

                    console.warn('Failed to connect', err)
                  })
                }}
              />
            )}
          </Navbar.Group>{' '}
          <Navbar.Group align={Alignment.RIGHT}>
            <Button
              minimal
              large
              icon="dashboard"
              text="Control Panel"
              onClick={() => {
                navigate(`/devices/${props.deviceID}/Control_Panel`)
              }}
              active={page === 'Control_Panel'}
            />
            <Button
              minimal
              large
              icon="cog"
              text="PID Panel"
              onClick={() => {
                navigate(`/devices/${props.deviceID}/PID_Panel`)
              }}
              active={page === 'PID_Panel'}
            />
             <Button
              minimal
              large
              icon="cog"
              text="Calibration Panel"
              onClick={() => {
                navigate(`/devices/${props.deviceID}/Cal_Panel`)
              }}
              active={page === 'Cal_Panel'}
            />
          </Navbar.Group>{' '}
        </div>
      </Navbar>
    </div>
  )
}
