import {describe, it, expect, vi} from 'vitest';
import {render, screen} from '@testing-library/react';
import {MowerStatus} from './MowerStatus.tsx';

const mockHighLevelStatus = vi.fn();
vi.mock('../hooks/useHighLevelStatus.ts', () => ({
    useHighLevelStatus: () => mockHighLevelStatus(),
}));

// Plan 02-07: useDockMatch subscribes to /dock_match/* via the Go relay
// (useWS WebSocket). Stub it out in unit tests so render doesn't try to
// open a real WebSocket connection.
vi.mock('../hooks/useDockMatch.ts', () => ({
    useDockMatch: () => ({pose: null, confidence: null, lastMessageAt: null}),
}));

describe('MowerStatus', () => {
    it('displays idle state', () => {
        mockHighLevelStatus.mockReturnValue({
            highLevelStatus: {StateName: 'IDLE', GpsQualityPercent: 0.95, BatteryPercent: 0.75, IsCharging: false},
        });
        render(<MowerStatus/>);
        expect(screen.getByText('Idle')).toBeInTheDocument();
        expect(screen.getByText('95%')).toBeInTheDocument();
        expect(screen.getByText('75%')).toBeInTheDocument();
    });

    it('displays mowing state', () => {
        mockHighLevelStatus.mockReturnValue({
            highLevelStatus: {StateName: 'MOWING', GpsQualityPercent: 1.0, BatteryPercent: 0.5, IsCharging: false},
        });
        render(<MowerStatus/>);
        expect(screen.getByText('Mowing')).toBeInTheDocument();
        expect(screen.getByText('100%')).toBeInTheDocument();
        expect(screen.getByText('50%')).toBeInTheDocument();
    });

    it('handles undefined values gracefully', () => {
        mockHighLevelStatus.mockReturnValue({
            highLevelStatus: {},
        });
        render(<MowerStatus/>);
        expect(screen.getByText('Offline')).toBeInTheDocument();
        expect(screen.getAllByText('0%')).toHaveLength(2);
    });
});
