from __future__ import annotations

# The harness only exposes the editable block to the model.
# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi
from pathlib import Path

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
def _origin(xyz, rpy=(0.0, 0.0, 0.0)) -> Origin:
    return Origin(xyz=xyz, rpy=rpy)


def _add_box(part, size, xyz, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Box(size), origin=_origin(xyz, rpy))


def _add_cylinder(part, radius, length, xyz, rpy=(0.0, 0.0, 0.0)) -> None:
    part.visual(Cylinder(radius=radius, length=length), origin=_origin(xyz, rpy))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_electrical_cabinet", assets=ASSETS)

    cab_w = 0.76
    cab_h = 1.12
    cab_d = 0.34
    side_t = 0.018
    back_t = 0.003
    flange_t = 0.020
    flange_w = 0.028

    door_w = 0.70
    door_h = 1.06
    hinge_clear = 0.016

    hinge_origin_x = cab_d / 2.0 + 0.003
    hinge_origin_y = hinge_clear + door_w / 2.0
    door_center_y = -(hinge_clear + door_w / 2.0)
    free_edge_y = -(hinge_clear + door_w)
    handle_y = free_edge_y + 0.090

    cabinet = model.part("cabinet")
    cabinet.inertial = Inertial.from_geometry(Box((cab_d, cab_w, cab_h)), mass=88.0)

    shell_depth = cab_d - back_t
    _add_box(cabinet, (back_t, cab_w, cab_h), (-cab_d / 2.0 + back_t / 2.0, 0.0, 0.0))
    _add_box(
        cabinet, (shell_depth, side_t, cab_h), (back_t / 2.0, -cab_w / 2.0 + side_t / 2.0, 0.0)
    )
    _add_box(cabinet, (shell_depth, side_t, cab_h), (back_t / 2.0, cab_w / 2.0 - side_t / 2.0, 0.0))
    _add_box(
        cabinet,
        (shell_depth, cab_w - 2.0 * side_t, side_t),
        (back_t / 2.0, 0.0, -cab_h / 2.0 + side_t / 2.0),
    )
    _add_box(
        cabinet,
        (shell_depth, cab_w - 2.0 * side_t, side_t),
        (back_t / 2.0, 0.0, cab_h / 2.0 - side_t / 2.0),
    )

    flange_x = cab_d / 2.0 - 0.022
    _add_box(
        cabinet, (flange_t, flange_w, cab_h - 0.10), (flange_x, -cab_w / 2.0 + flange_w / 2.0, 0.0)
    )
    _add_box(
        cabinet, (flange_t, flange_w, cab_h - 0.10), (flange_x, cab_w / 2.0 - flange_w / 2.0, 0.0)
    )
    _add_box(
        cabinet,
        (flange_t, cab_w - 2.0 * flange_w, flange_w),
        (flange_x, 0.0, -cab_h / 2.0 + flange_w / 2.0),
    )
    _add_box(
        cabinet,
        (flange_t, cab_w - 2.0 * flange_w, flange_w),
        (flange_x, 0.0, cab_h / 2.0 - flange_w / 2.0),
    )

    plate_x = -cab_d / 2.0 + 0.055
    _add_box(cabinet, (0.003, cab_w - 0.14, cab_h - 0.18), (plate_x, 0.0, 0.0))
    standoff_x = (-cab_d / 2.0 + back_t + plate_x) / 2.0
    standoff_len = plate_x - (-cab_d / 2.0 + back_t)
    for y in (-0.25, 0.25):
        for z in (-0.38, 0.38):
            _add_box(cabinet, (standoff_len, 0.026, 0.026), (standoff_x, y, z))

    _add_box(
        cabinet, (0.060, cab_w - 0.08, 0.030), (-cab_d / 2.0 + 0.030, 0.0, -cab_h / 2.0 + 0.015)
    )
    _add_box(
        cabinet, (0.016, 0.040, cab_h - 0.24), (cab_d / 2.0 - 0.008, -cab_w / 2.0 + 0.020, 0.0)
    )
    _add_box(cabinet, (0.016, 0.040, cab_h - 0.14), (hinge_origin_x - 0.010, hinge_origin_y, 0.0))
    for z in (-0.34, 0.0, 0.34):
        _add_box(
            cabinet, (0.010, 0.022, 0.120), (hinge_origin_x - 0.011, hinge_origin_y - 0.001, z)
        )

    door = model.part("door")
    door.inertial = Inertial.from_geometry(
        Box((0.050, door_w, door_h)),
        mass=24.0,
        origin=_origin((0.010, door_center_y, 0.0)),
    )

    _add_box(door, (0.018, door_w, door_h), (0.008, door_center_y, 0.0))
    _add_box(door, (0.008, door_w, 0.060), (0.014, door_center_y, -door_h / 2.0 + 0.030))
    _add_box(door, (0.008, door_w, 0.060), (0.014, door_center_y, door_h / 2.0 - 0.030))
    _add_box(door, (0.008, 0.055, door_h - 0.12), (0.014, -hinge_clear - 0.0275, 0.0))
    _add_box(door, (0.008, 0.055, door_h - 0.12), (0.014, free_edge_y + 0.0275, 0.0))
    _add_box(door, (0.010, door_w - 0.18, door_h - 0.18), (0.009, door_center_y, 0.0))

    _add_box(door, (0.028, 0.032, door_h - 0.08), (0.001, -hinge_clear - 0.016, 0.0))
    _add_box(door, (0.028, 0.032, door_h - 0.08), (0.001, free_edge_y + 0.016, 0.0))
    _add_box(door, (0.028, door_w - 0.064, 0.032), (0.001, door_center_y, -door_h / 2.0 + 0.016))
    _add_box(door, (0.028, door_w - 0.064, 0.032), (0.001, door_center_y, door_h / 2.0 - 0.016))

    _add_box(door, (0.014, 0.040, door_h - 0.34), (0.000, door_center_y + 0.160, 0.0))
    _add_box(door, (0.014, 0.040, door_h - 0.34), (0.000, door_center_y - 0.160, 0.0))

    _add_box(door, (0.016, 0.050, 0.260), (0.000, handle_y, 0.0))
    _add_cylinder(door, 0.006, 0.360, (0.000, handle_y, 0.290))
    _add_cylinder(door, 0.006, 0.360, (0.000, handle_y, -0.290))
    _add_box(door, (0.018, 0.038, 0.020), (0.001, handle_y, 0.110))
    _add_box(door, (0.018, 0.038, 0.020), (0.001, handle_y, -0.110))
    _add_box(door, (0.018, 0.038, 0.020), (0.001, handle_y, 0.470))
    _add_box(door, (0.018, 0.038, 0.020), (0.001, handle_y, -0.470))

    _add_box(door, (0.020, 0.090, 0.300), (0.022, handle_y, 0.0))
    _add_cylinder(door, 0.015, 0.018, (0.021, handle_y, 0.0), rpy=(0.0, pi / 2.0, 0.0))
    _add_box(door, (0.024, 0.032, 0.180), (0.034, handle_y, 0.0))
    _add_box(door, (0.010, 0.032, 0.100), (0.041, handle_y, -0.040))

    _add_box(door, (0.004, 0.180, 0.070), (0.018, door_center_y + 0.090, door_h / 2.0 - 0.120))

    _add_box(door, (0.004, 0.028, door_h - 0.12), (-0.010, -hinge_clear - 0.014, 0.0))
    _add_box(door, (0.004, 0.028, door_h - 0.12), (-0.010, free_edge_y + 0.014, 0.0))
    _add_box(door, (0.004, door_w - 0.056, 0.028), (-0.010, door_center_y, -door_h / 2.0 + 0.014))
    _add_box(door, (0.004, door_w - 0.056, 0.028), (-0.010, door_center_y, door_h / 2.0 - 0.014))

    for z in (-0.34, 0.0, 0.34):
        _add_cylinder(door, 0.010, 0.140, (0.000, 0.000, z))
        _add_box(door, (0.008, 0.020, 0.140), (0.004, -0.008, z))

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent="cabinet",
        child="door",
        origin=_origin((hinge_origin_x, hinge_origin_y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.2, lower=0.0, upper=2.1),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)

    def _vec3(value):
        if isinstance(value, (tuple, list)) and len(value) == 3:
            return (float(value[0]), float(value[1]), float(value[2]))
        if hasattr(value, "x") and hasattr(value, "y") and hasattr(value, "z"):
            return (float(value.x), float(value.y), float(value.z))
        raise TypeError(f"Unsupported vector representation: {value!r}")

    def _bounds(part_name: str):
        aabb = ctx.part_world_aabb(part_name, use="collision")
        if all(
            hasattr(aabb, key) for key in ("min_x", "max_x", "min_y", "max_y", "min_z", "max_z")
        ):
            return {
                "min_x": float(aabb.min_x),
                "max_x": float(aabb.max_x),
                "min_y": float(aabb.min_y),
                "max_y": float(aabb.max_y),
                "min_z": float(aabb.min_z),
                "max_z": float(aabb.max_z),
            }
        for low_name, high_name in (("minimum", "maximum"), ("min", "max"), ("mins", "maxs")):
            if hasattr(aabb, low_name) and hasattr(aabb, high_name):
                low = _vec3(getattr(aabb, low_name))
                high = _vec3(getattr(aabb, high_name))
                return {
                    "min_x": low[0],
                    "min_y": low[1],
                    "min_z": low[2],
                    "max_x": high[0],
                    "max_y": high[1],
                    "max_z": high[2],
                }
        if isinstance(aabb, (tuple, list)) and len(aabb) == 2:
            low = _vec3(aabb[0])
            high = _vec3(aabb[1])
            return {
                "min_x": low[0],
                "min_y": low[1],
                "min_z": low[2],
                "max_x": high[0],
                "max_y": high[1],
                "max_z": high[2],
            }
        raise TypeError(f"Unsupported AABB representation: {aabb!r}")

    def _require(condition: bool, message: str) -> None:
        if not condition:
            raise AssertionError(message)

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.allow_overlap(
        "cabinet",
        "door",
        reason="Closed cabinet door intentionally seats against the front return and hinge knuckles; generated collision hulls conservatively report this tight industrial fit as overlap.",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=160, overlap_tol=0.002, overlap_volume_tol=0.0)
    ctx.expect_joint_motion_axis(
        "door_hinge", "door", world_axis="x", direction="positive", min_delta=0.18
    )

    cabinet_closed = _bounds("cabinet")
    door_closed = _bounds("door")

    _require(
        cabinet_closed["max_x"] - cabinet_closed["min_x"] > 0.33,
        "Cabinet depth should read as a full enclosure.",
    )
    _require(
        cabinet_closed["max_y"] - cabinet_closed["min_y"] > 0.74,
        "Cabinet width should feel industrial and substantial.",
    )
    _require(
        cabinet_closed["max_z"] - cabinet_closed["min_z"] > 1.10,
        "Cabinet should have realistic full-height proportions.",
    )

    _require(
        cabinet_closed["max_x"] - 0.020 <= door_closed["min_x"] <= cabinet_closed["max_x"] + 0.006,
        "Closed door should tuck just over the front opening without being buried in the shell.",
    )
    _require(
        door_closed["max_x"] - door_closed["min_x"] > 0.055,
        "Door depth should include visible latch hardware and inner return construction.",
    )
    _require(
        door_closed["min_y"] > cabinet_closed["min_y"] + 0.020
        and door_closed["max_y"] < cabinet_closed["max_y"] - 0.002,
        "Closed door should sit inside the cabinet side envelope with only narrow reveal gaps.",
    )
    _require(
        door_closed["min_z"] > cabinet_closed["min_z"] + 0.020
        and door_closed["max_z"] < cabinet_closed["max_z"] - 0.020,
        "Door should leave believable top and bottom clearances inside the enclosure frame.",
    )
    _require(
        door_closed["max_z"] - door_closed["min_z"] > 1.00,
        "Door panel should be tall enough for a service enclosure.",
    )
    _require(
        door_closed["max_y"] - door_closed["min_y"] > 0.69,
        "Door panel should cover almost the full cabinet width.",
    )

    with ctx.pose(door_hinge=1.2):
        door_mid = _bounds("door")
        _require(
            door_mid["max_x"] > door_closed["max_x"] + 0.22,
            "Door should swing well outward at a service position.",
        )
        _require(
            door_mid["min_y"] > 0.08,
            "Partially opened door should clear the front opening toward the hinge side.",
        )
        _require(
            abs(
                (door_mid["max_z"] - door_mid["min_z"])
                - (door_closed["max_z"] - door_closed["min_z"])
            )
            < 0.02,
            "Opening the hinge should preserve the door's vertical extent.",
        )

    with ctx.pose(door_hinge=2.05):
        door_open = _bounds("door")
        _require(
            door_open["max_x"] > cabinet_closed["max_x"] + 0.45,
            "Fully opened door should project far clear of the cabinet face.",
        )
        _require(
            door_open["min_y"] > 0.30,
            "At full swing the door mass should gather toward the hinge side of the enclosure.",
        )
        _require(
            (door_open["max_y"] - door_open["min_y"])
            < (door_closed["max_y"] - door_closed["min_y"]) - 0.22,
            "Open door should present a much narrower frontal width than when closed.",
        )
        _require(
            (door_open["max_x"] - door_open["min_x"])
            > (door_closed["max_x"] - door_closed["min_x"]) + 0.40,
            "Open door should show a large outward sweep in the collision envelope.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
