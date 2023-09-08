import * as React from 'react'
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
import { Button } from '@electricui/components-desktop-blueprint'
import classnames from 'classnames'
import { parse } from 'csv/sync'
import { readFileSync } from 'fs'
import { IconNames } from '@blueprintjs/icons'
import { Composition } from 'atomic-layout'

import { useDeviceMetadataKey } from '@electricui/components-core'

import { NumberInput } from '@electricui/components-desktop-blueprint'

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
  Button
Calibrate
Slider
`

export const CalPage = (props: RouteComponentProps) => {
  const deviceID = useDeviceMetadataKey('name') ?? 'BumpRig Module'

  return (
    <Composition templateCols="1fr" gap={10} areas={layoutDescription}>
      {Areas => (
        <React.Fragment>
          <Areas.Chart templateCols="1fr">
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
                    'v_Sensor',
                    'n_Sensor',
                    'CalMax',
                    'CalMin',
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
          <Areas.Button>
            <Button
              fill
              icon={IconNames.SERIES_CONFIGURATION}
              intent="warning"
              writer={state => {
                state.zero_stroke = 1
              }}
            >
              Calibrate
            </Button>
          </Areas.Button>

          <Areas.Calibrate justifyItems="center">
            <Card>
              <div style={{ textAlign: 'left', marginBottom: '1em' }}>
                <b>Stroke Sensor Length (mm)</b>
              </div>
              <div
                style={{
                  justifyContent: 'center',
                  alignItems: 'center',
                  marginBottom: '1em',
                }}
              >
                <NumberInput
                  accessor="x_str_sen"
                  min={75}
                  max={250}
                  large
                  leftIcon={IconNames.MAXIMIZE}
                  intent={Intent.PRIMARY}
                />
              </div>
              <div style={{ textAlign: 'left', marginBottom: '1em' }}>
                <b>Stroke Sensor Slope (mm/V)</b>
              </div>
              <div style={{ marginBottom: '1em' }}>
                <NumberInput
                  accessor="r_str_cal"
                  large
                  leftIcon={IconNames.CHART}
                  intent={Intent.PRIMARY}
                />
              </div>
              <div style={{ textAlign: 'left', marginBottom: '1em' }}>
                <b>Stroke Sensor Offset (mm)</b>
              </div>
              <div style={{ marginBottom: '1em' }}>
                <NumberInput
                  accessor="n_str_cal"
                  large
                  leftIcon={IconNames.ARROWS_VERTICAL}
                  intent={Intent.PRIMARY}
                />
                <Statistic
                  accessor="n_Sensor"
                  label="Stroke Sensor"
                  suffix="Count"
                  color={Colors.BLUE1}
                />
                <Statistic
                  accessor="v_Sensor"
                  label="Stroke Sensor"
                  suffix="V"
                  color={Colors.BLUE1}
                />
                <Statistic
                  accessor="x_position"
                  label="Stroke Sensor"
                  suffix="mm"
                  color={Colors.BLUE1}
                />
                <Statistic
                  accessor="CalMax"
                  label="Cal Max"
                  suffix="mm"
                  color={Colors.BLUE1}
                />
                <Statistic
                  accessor="CalMin"
                  label="Cal Min"
                  suffix="mm"
                  color={Colors.BLUE1}
                />
              </div>
            </Card>
          </Areas.Calibrate>
          <Areas.Slider>
            <Card>
              <div>Set Point Override</div>
              <div style={{ margin: 20 }}>
                <Slider min={0} max={200} stepSize={5} labelStepSize={20}>
                  <Slider.Handle accessor="set_point" />
                </Slider>
              </div>
            </Card>
          </Areas.Slider>
        </React.Fragment>
      )}
    </Composition>
  )
}
