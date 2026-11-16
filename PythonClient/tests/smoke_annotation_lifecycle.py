"""Regression for dynamic annotation cleanup and for annotation API calls made while the simulation runs
(issue #176). Use a disposable simulator."""
import argparse
import time
import uuid
import cosysairsim as airsim


def check_regex_and_annotation_calls(client, prefix, greyscale_layer):
    # The setters used to crash the simulator (assertion IsInGameThread) as they were run on the RPC thread.
    start = time.time()
    assert client.simSetSegmentationObjectID(".*" + prefix + ".*", 1500, True)
    print('regex segmentation update took %.3f s' % (time.time() - start))
    start = time.time()
    client.simSetSegmentationObjectID(".*", 0, True)
    print('regex segmentation update of all objects took %.3f s' % (time.time() - start))
    if greyscale_layer:
        objects = client.simListAnnotationObjects(greyscale_layer)
        assert len(objects) > 0, 'Annotation layer %s has no objects' % greyscale_layer
        assert client.simSetAnnotationObjectValue(greyscale_layer, objects[0], 0.5)
        assert abs(client.simGetAnnotationObjectValue(greyscale_layer, objects[0]) - 0.5) < 1e-3
        start = time.time()
        assert client.simSetAnnotationObjectValue(greyscale_layer, ".*", 0.25, True)
        print('regex greyscale annotation update took %.3f s' % (time.time() - start))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--greyscale-layer', default='', help='name of a greyscale annotation layer to test (optional)')
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
            if cycle == 0:
                check_regex_and_annotation_calls(client, prefix, args.greyscale_layer)
        finally:
            assert client.simDestroyObject(name), name
        time.sleep(0.5)
        assert not any(prefix in key for key in client.simListInstanceSegmentationObjects())
        assert client.simGetSegmentationObjectID(key) == -1
    print('PASS: two spawn/render/update/destroy cycles, regex and annotation updates, no stale annotation names or IDs')


if __name__ == '__main__':
    main()
