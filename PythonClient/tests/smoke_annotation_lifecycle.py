"""Regression for dynamic annotation cleanup. Use a disposable simulator."""
import argparse
import time
import uuid
import cosysairsim as airsim


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, required=True)
    args = parser.parse_args()
    client = airsim.VehicleClient(port=args.port, timeout_value=15)
    assert client.ping()
    assert 'Cube' in client.simListAssets(), 'Test level must include a Cube asset'
    prefix = 'AnnotationAudit_' + uuid.uuid4().hex[:8]
    for cycle in range(2):
        name = client.simSpawnObject(prefix, 'Cube', airsim.Pose(airsim.Vector3r(10, 0, -10)),
                                     airsim.Vector3r(1, 1, 1), False, False)
        try:
            names = [key for key in client.simListInstanceSegmentationObjects() if prefix in key]
            assert len(names) == 1, names
            key = names[0]
            # The render proxy is created after the initial spawn registration.
            time.sleep(1)
            assert client.simSetSegmentationObjectID(key, 1000 + cycle)
            assert client.simGetSegmentationObjectID(key) == 1000 + cycle
        finally:
            assert client.simDestroyObject(name), name
        time.sleep(0.5)
        assert not any(prefix in key for key in client.simListInstanceSegmentationObjects())
        assert client.simGetSegmentationObjectID(key) == -1
    print('PASS: two spawn/render/update/destroy cycles, no stale annotation names or IDs')


if __name__ == '__main__':
    main()
