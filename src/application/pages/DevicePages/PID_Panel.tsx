import * as React from 'react'
import { useCallback, useState, useMemo, useEffect } from 'react'
import { RouteComponentProps } from '@reach/router'
import {
  ButtonGroup,
  Card,
  Colors,
  Icon,
  NumericInput,
  Intent,
  Tree,
  TreeNodeInfo,
  TreeEventHandler,
} from '@blueprintjs/core'
import {
  ChartContainer,
  LineChart,
  RealTimeDomain,
  VerticalAxis,
  TimeAxis,
  ZoomBrush,
} from '@electricui/charts'
import { MessageDataSource } from '@electricui/core-timeseries'
import {
  useDeviceID,
  useSendMessage,
  IntervalRequester,
  useHardwareState,
} from '@electricui/components-core'
import {
  Slider,
  Statistic,
  Statistics,
} from '@electricui/components-desktop-blueprint'
import { OPEN_DIALOG_IPC_EVENT } from '@electricui/utility-electron'
import { ipcRenderer, OpenDialogOptions, OpenDialogReturnValue } from 'electron'
import { Button, Callout, ProgressBar } from '@blueprintjs/core'
import classnames from 'classnames'
import { parse } from 'csv/sync'
import { readFileSync } from 'fs'
import { IconNames } from '@blueprintjs/icons'
import { Composition } from 'atomic-layout'

import { useDeviceMetadataKey } from '@electricui/components-core'

// Add the `csv` package to package.json
import './tree-caret-override.css'
import { CancellationToken, Message } from '@electricui/core'

const strokeMeasuredDataSource = new MessageDataSource('x_position')
const setPointDataSource = new MessageDataSource('set_point')
const KpDataSource = new MessageDataSource('n_Kp')
const KiDataSource = new MessageDataSource('n_Ki')
const KdDataSource = new MessageDataSource('n_Kd')
const errorDataSource = new MessageDataSource('n_err')
const previouserrorDataSource = new MessageDataSource('n_prev_err')
const integralDataSource = new MessageDataSource('n_intl')
const derivativeDataSource = new MessageDataSource('n_der')
const controlsignalDataSource = new MessageDataSource('n_cont')

/**
 * Full width file picker
 */

const layoutDescription = `
  Chart
  Measure
  Measure2
  Slider 
  Slider2
  Slider3
  Slider4
`

export const PIDPage = (props: RouteComponentProps) => {
  const deviceID = useDeviceMetadataKey('name') ?? 'BumpRig Module'

  return (
    <Composition templateCols="1fr" gap={10} areas={layoutDescription}>
      {Areas => (
        <React.Fragment>
          <Areas.Chart>
            <Card>
              <div style={{ textAlign: 'center', marginBottom: '1em' }}>
                <IntervalRequester
                  variables={[
                    'set_point',
                    'x_position',
                    'n_Kp',
                    'n_Ki',
                    'n_Kd',
                    'n_err',
                    'n_prev_err',
                    'n_intl',
                    'n_der',
                    'n_cont',
                  ]}
                  interval={100}
                />
                <div style={{ textAlign: 'center', marginBottom: '1em' }}>
                  <b>Set Point</b> {deviceID}
                </div>
                <ChartContainer height={250}>
                  <LineChart
                    dataSource={setPointDataSource}
                    color={Colors.RED5}
                    lineWidth={5}
                  />
                  <LineChart
                    dataSource={strokeMeasuredDataSource}
                    color={Colors.BLUE5}
                    lineWidth={5}
                  />
                  <RealTimeDomain window={10000} yMin={0} yMax={200} />
                  <TimeAxis label="Time (Seconds)" />
                  <VerticalAxis label="Stroke Position (mm)" />
                  <ZoomBrush />
                </ChartContainer>
              </div>
            </Card>
          </Areas.Chart>
          <Areas.Measure>
            <Card>
              <Statistics>
                <Statistic accessor="n_Kp" label="Kp" color={Colors.BLUE1} />

                <Statistic accessor="n_Ki" label="Ki" color={Colors.BLUE3} />

                <Statistic accessor="n_Kd" label="Kd" color={Colors.BLUE5} />

                <Statistic accessor="n_err" label="Error" color={Colors.RED1} />

                <Statistic
                  accessor="n_prev_err"
                  label="Prev Error"
                  color={Colors.RED3}
                />
              </Statistics>
            </Card>
          </Areas.Measure>
          <Areas.Measure2>
            <Card>
              <Statistics>
                <Statistic
                  accessor="n_intl"
                  label="Integral"
                  color={Colors.RED5}
                />
                <Statistic
                  accessor="n_der"
                  label="Derivative"
                  color={Colors.RED5}
                />
                <Statistic
                  accessor="x_position"
                  label="Position"
                  suffix= "mm"
                  color={Colors.GREEN3}
                />

                <Statistic
                  accessor="n_cont"
                  label="Control Signal"
                  color={Colors.GREEN1}
                />
              </Statistics>
            </Card>
          </Areas.Measure2>
          <Areas.Slider>
            <Card>
              <div>Set Point Override</div>
              <div style={{ margin: 20 }}>
                <Slider sendOnlyOnRelease={false} defaultTrackIntent='primary' min={0} max={200} stepSize={5} labelStepSize={20}>
                  <Slider.Handle accessor="set_point" intentBefore='success' />
                </Slider>
              </div>
            </Card>
          </Areas.Slider>
          <Areas.Slider2>
            <Card>
              <div>Kp Override</div>
              <div style={{ margin: 20 }}>
                <Slider sendOnlyOnRelease={false} defaultTrackIntent='primary' min={0} max={5} stepSize={0.05} labelStepSize={1}>
                  <Slider.Handle accessor="n_Kp" intentBefore='danger'/>
                </Slider>
              </div>
            </Card>
          </Areas.Slider2>
          <Areas.Slider3>
            <Card>
              <div>Ki Override</div>
              <div style={{ margin: 20 }}>
                <Slider sendOnlyOnRelease={false} defaultTrackIntent='warning' min={0} max={0.1} stepSize={0.005} labelStepSize={0.01}>
                  <Slider.Handle accessor="n_Ki" intentBefore='danger' />
                </Slider>
              </div>
            </Card>
          </Areas.Slider3>
          <Areas.Slider4>
            <Card>
              <div>Kd Override</div>
              <div style={{ margin: 20 }}>
                <Slider sendOnlyOnRelease={false} defaultTrackIntent='warning' min={0} max={2} stepSize={0.1} labelStepSize={1}>
                  <Slider.Handle accessor="n_Kd" intentBefore='danger' />
                </Slider>
              </div>
            </Card>
          </Areas.Slider4>
          
        </React.Fragment>
      )}
    </Composition>
    
  )
}
