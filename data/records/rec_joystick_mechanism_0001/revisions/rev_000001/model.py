from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

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
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_X = 0.214
BASE_Y = 0.178
BASE_T = 0.012
PIVOT_Z = 0.058

SERVICE_COVER_X = 0.030
SERVICE_COVER_Y = 0.060
SERVICE_COVER_T = 0.004
SERVICE_COVER_POS_X = 0.086

OPENING_R = 0.039
LOWER_BOOT_OR = 0.055
LOWER_BOOT_IR = 0.040
UPPER_BOOT_OR = 0.052
UPPER_BOOT_IR = 0.036

PITCH_BRACKET_Y = 0.070
PITCH_BRACKET_X = 0.048
PITCH_BRACKET_T = 0.012
PITCH_BRACKET_H = 0.062
PITCH_TRUNNION_R = 0.0072
PITCH_BUSHING_OR = 0.0090
PITCH_BUSHING_LEN = PITCH_BRACKET_T
CAP_T = 0.008

OUTER_ARM_X = 0.014
OUTER_ARM_Y = 0.016
OUTER_ARM_HEIGHT = 0.072
OUTER_ARM_OFFSET_X = 0.038
ROLL_TRUNNION_R = 0.0062
ROLL_BUSHING_OR = 0.0090
ROLL_BUSHING_LEN = 0.012

INNER_CHEEK_X = 0.014
INNER_CHEEK_Y = 0.012
INNER_CHEEK_H = 0.074
INNER_CHEEK_OFFSET_Y = 0.024

SHAFT_R = 0.0085


def _box_center(
    dx: float,
    dy: float,
    dz: float,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
    fillet: float | None = None,
):
    box = cq.Workplane("XY").box(dx, dy, dz)
    if fillet:
        box = box.edges("|Z").fillet(fillet)
    return box.translate(center)


def _box_bottom(
    dx: float,
    dy: float,
    dz: float,
    center_xy: tuple[float, float] = (0.0, 0.0),
    z0: float = 0.0,
    fillet: float | None = None,
):
    box = (
        cq.Workplane("XY")
        .box(dx, dy, dz, centered=(True, True, False))
        .translate((center_xy[0], center_xy[1], z0))
    )
    if fillet:
        box = box.edges("|Z").fillet(fillet)
    return box


def _cyl(axis: str, radius: float, length: float, center: tuple[float, float, float] = (0.0, 0.0, 0.0)):
    half = length / 2.0
    if axis == "x":
        return cq.Workplane("YZ").circle(radius).extrude(half, both=True).translate(center)
    if axis == "y":
        return cq.Workplane("XZ").circle(radius).extrude(half, both=True).translate(center)
    return cq.Workplane("XY").circle(radius).extrude(half, both=True).translate(center)


def _ring(
    axis: str,
    outer_radius: float,
    inner_radius: float,
    length: float,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
):
    return _cyl(axis, outer_radius, length, center=center).cut(
        _cyl(axis, inner_radius, length + 0.002, center=center)
    )


def _socket_head_pattern(
    body,
    points: list[tuple[float, float]],
    radius: float,
    height: float,
    z_center: float,
):
    result = body
    for x, y in points:
        result = result.union(_cyl("z", radius, height, center=(x, y, z_center)))
    return result


def _make_base_frame():
    base = _box_bottom(BASE_X, BASE_Y, BASE_T, z0=0.0, fillet=0.0025)

    for x_sign in (-1.0, 1.0):
        x = x_sign * SERVICE_COVER_POS_X
        base = base.cut(
            _box_bottom(
                SERVICE_COVER_X - 0.002,
                SERVICE_COVER_Y - 0.004,
                0.0015,
                center_xy=(x, 0.0),
                z0=BASE_T - 0.0015,
            )
        )
        for sy in (-0.022, 0.022):
            base = base.cut(_cyl("z", 0.0019, 0.010, center=(x, sy, BASE_T / 2)))

    base = base.cut(_cyl("z", OPENING_R, BASE_T + 0.020, center=(0.0, 0.0, BASE_T / 2)))

    for y_sign in (-1.0, 1.0):
        y = y_sign * PITCH_BRACKET_Y
        bracket = _box_bottom(
            PITCH_BRACKET_X,
            PITCH_BRACKET_T,
            PITCH_BRACKET_H,
            center_xy=(0.0, y),
            z0=BASE_T,
            fillet=0.0012,
        )
        bracket = bracket.cut(
            _box_bottom(
                0.022,
                PITCH_BRACKET_T + 0.003,
                0.026,
                center_xy=(0.0, y),
                z0=BASE_T + 0.018,
            )
        )
        base = base.union(bracket)
        base = base.cut(_cyl("y", PITCH_BUSHING_OR + 0.0002, PITCH_BRACKET_T + 0.003, center=(0.0, y, PIVOT_Z)))

        for x_sign in (-1.0, 1.0):
            gusset = (
                cq.Workplane("XZ")
                .polyline(
                    [
                        (x_sign * 0.022, BASE_T),
                        (x_sign * 0.022, BASE_T + 0.004),
                        (x_sign * 0.008, PIVOT_Z - 0.010),
                        (x_sign * 0.008, BASE_T),
                    ]
                )
                .close()
                .extrude(0.008, both=True)
                .translate((0.0, y - y_sign * 0.002, 0.0))
            )
            base = base.union(gusset)

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            x = x_sign * 0.060
            y = y_sign * 0.034
            tower = _box_bottom(0.012, 0.012, 0.020, center_xy=(x, y), z0=BASE_T, fillet=0.001)
            tower = tower.cut(_cyl("x", 0.0018, 0.016, center=(x, y, BASE_T + 0.016)))
            base = base.union(tower)

    for x_sign in (-1.0, 1.0):
        for y_sign in (-1.0, 1.0):
            base = base.union(_cyl("z", 0.008, 0.006, center=(x_sign * 0.080, y_sign * 0.058, -0.003)))
    return base


def _make_service_cover(x_sign: float):
    cover = _box_bottom(SERVICE_COVER_X, SERVICE_COVER_Y, 0.003, z0=0.0, fillet=0.0012)
    cover = _socket_head_pattern(
        cover,
        [(0.0, -0.022), (0.0, 0.022)],
        radius=0.0022,
        height=0.0015,
        z_center=0.00375,
    )
    cover = cover.union(
        _box_bottom(0.006, 0.016, 0.002, center_xy=(x_sign * 0.008, 0.0), z0=0.003)
    )
    return cover


def _make_trunnion_cap(outward_sign: float):
    cap = _box_center(0.050, CAP_T, 0.028, fillet=0.0012)
    cap = cap.cut(_cyl("y", 0.0082, CAP_T + 0.003, center=(0.0, 0.0, 0.0)))
    cap = cap.cut(_box_center(0.018, CAP_T + 0.003, 0.010, center=(0.0, 0.0, -0.009)))
    for x in (-0.018, 0.018):
        cap = cap.union(
            _cyl(
                "y",
                0.0024,
                0.002,
                center=(x, outward_sign * (CAP_T / 2 + 0.001), 0.008),
            )
        )
    return cap


def _make_pitch_bushing():
    bushing = _ring("y", PITCH_BUSHING_OR, PITCH_TRUNNION_R + 0.0004, PITCH_BUSHING_LEN)
    bushing = bushing.union(_ring("y", 0.0105, PITCH_TRUNNION_R + 0.0004, 0.004))
    return bushing


def _make_lower_boot_ring():
    ring = _ring("z", LOWER_BOOT_OR, LOWER_BOOT_IR, 0.003)
    for angle_deg in range(0, 360, 45):
        angle = math.radians(angle_deg)
        x = 0.048 * math.cos(angle)
        y = 0.048 * math.sin(angle)
        ring = ring.union(_cyl("z", 0.0022, 0.002, center=(x, y, 0.0025)))
    return ring


def _make_upper_boot_clamp():
    clamp = _ring("z", UPPER_BOOT_OR, UPPER_BOOT_IR, 0.003)
    for angle_deg in (45, 135, 225, 315):
        angle = math.radians(angle_deg)
        x = 0.045 * math.cos(angle)
        y = 0.045 * math.sin(angle)
        clamp = clamp.union(_cyl("z", 0.0030, 0.003, center=(x, y, 0.0015)))
    for x_sign in (-1.0, 1.0):
        clamp = clamp.union(_box_center(0.012, 0.008, 0.003, center=(x_sign * 0.043, 0.0, 0.0)))
    return clamp


def _make_outer_yoke():
    yoke = _box_bottom(0.008, 0.008, 0.056, center_xy=(-OUTER_ARM_OFFSET_X, 0.0), z0=-0.014, fillet=0.0008)
    yoke = yoke.union(
        _box_bottom(0.008, 0.008, 0.056, center_xy=(OUTER_ARM_OFFSET_X, 0.0), z0=-0.014, fillet=0.0008)
    )
    yoke = yoke.union(_box_bottom(0.068, 0.008, 0.008, center_xy=(0.0, 0.026), z0=0.028, fillet=0.0008))
    yoke = yoke.union(_box_bottom(0.068, 0.008, 0.008, center_xy=(0.0, -0.026), z0=0.028, fillet=0.0008))
    yoke = yoke.union(_box_bottom(0.056, 0.008, 0.006, center_xy=(0.0, 0.026), z0=-0.014, fillet=0.0008))
    yoke = yoke.union(_box_bottom(0.056, 0.008, 0.006, center_xy=(0.0, -0.026), z0=-0.014, fillet=0.0008))
    yoke = yoke.union(_box_bottom(0.014, 0.032, 0.008, center_xy=(0.0, 0.050), z0=-0.004, fillet=0.0008))
    yoke = yoke.union(_box_bottom(0.014, 0.032, 0.008, center_xy=(0.0, -0.050), z0=-0.004, fillet=0.0008))
    yoke = yoke.union(_cyl("y", PITCH_TRUNNION_R, PITCH_BUSHING_LEN, center=(0.0, PITCH_BRACKET_Y, 0.0)))
    yoke = yoke.union(_cyl("y", PITCH_TRUNNION_R, PITCH_BUSHING_LEN, center=(0.0, -PITCH_BRACKET_Y, 0.0)))
    yoke = yoke.union(_box_bottom(0.012, 0.010, 0.012, center_xy=(-OUTER_ARM_OFFSET_X, 0.0), z0=-0.006, fillet=0.0008))
    yoke = yoke.union(_box_bottom(0.012, 0.010, 0.012, center_xy=(OUTER_ARM_OFFSET_X, 0.0), z0=-0.006, fillet=0.0008))
    yoke = yoke.cut(_cyl("x", ROLL_BUSHING_OR + 0.0002, 0.016, center=(-OUTER_ARM_OFFSET_X, 0.0, 0.0)))
    yoke = yoke.cut(_cyl("x", ROLL_BUSHING_OR + 0.0002, 0.016, center=(OUTER_ARM_OFFSET_X, 0.0, 0.0)))
    for y_sign in (-1.0, 1.0):
        yoke = yoke.union(_box_center(0.010, 0.006, 0.008, center=(0.0, y_sign * 0.040, 0.012)))
        yoke = yoke.cut(_cyl("x", 0.0018, 0.012, center=(0.0, y_sign * 0.040, 0.012)))

    return yoke


def _make_roll_bearing():
    bearing = _ring("x", ROLL_BUSHING_OR, ROLL_TRUNNION_R + 0.0004, ROLL_BUSHING_LEN)
    bearing = bearing.union(_ring("x", 0.0105, ROLL_TRUNNION_R + 0.0004, 0.004))
    return bearing


def _make_inner_carrier():
    carrier = _box_bottom(0.050, 0.006, 0.044, center_xy=(0.0, -0.016), z0=-0.010, fillet=0.0008)
    carrier = carrier.union(
        _box_bottom(0.050, 0.006, 0.044, center_xy=(0.0, 0.016), z0=-0.010, fillet=0.0008)
    )
    carrier = carrier.union(_box_bottom(0.012, 0.038, 0.008, center_xy=(0.0, 0.0), z0=0.028, fillet=0.0008))
    carrier = carrier.union(_ring("z", 0.015, SHAFT_R + 0.0008, 0.010, center=(0.0, 0.0, -0.002)))
    carrier = carrier.union(_box_bottom(0.012, 0.016, 0.010, center_xy=(0.0, -0.016), z0=-0.004, fillet=0.0008))
    carrier = carrier.union(_box_bottom(0.012, 0.016, 0.010, center_xy=(0.0, 0.016), z0=-0.004, fillet=0.0008))
    carrier = carrier.union(_box_bottom(0.012, 0.010, 0.012, center_xy=(-0.026, 0.0), z0=-0.006, fillet=0.0008))
    carrier = carrier.union(_box_bottom(0.012, 0.010, 0.012, center_xy=(0.026, 0.0), z0=-0.006, fillet=0.0008))
    carrier = carrier.union(_cyl("x", ROLL_TRUNNION_R, 0.012, center=(-OUTER_ARM_OFFSET_X, 0.0, 0.0)))
    carrier = carrier.union(_cyl("x", ROLL_TRUNNION_R, 0.012, center=(OUTER_ARM_OFFSET_X, 0.0, 0.0)))
    for y_sign in (-1.0, 1.0):
        carrier = carrier.union(_box_center(0.010, 0.010, 0.010, center=(0.0, y_sign * 0.030, 0.014)))
        carrier = carrier.cut(_cyl("x", 0.0018, 0.012, center=(0.0, y_sign * 0.030, 0.014)))

    return carrier


def _make_control_shaft():
    shaft = _cyl("z", SHAFT_R, 0.206, center=(0.0, 0.0, 0.103))
    shaft = shaft.union(_cyl("z", 0.0070, 0.050, center=(0.0, 0.0, -0.020)))
    shaft = shaft.union(_ring("z", 0.018, 0.0095, 0.004, center=(0.0, 0.0, -0.002)))
    shaft = shaft.union(_cyl("z", 0.0120, 0.014, center=(0.0, 0.0, 0.165)))
    shaft = shaft.union(_cyl("z", 0.0050, 0.026, center=(0.0, 0.0, 0.219)))
    shaft = shaft.cut(_cyl("x", 0.0022, 0.020, center=(0.0, 0.0, 0.193)))
    return shaft


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gimbal_joystick_mechanism", assets=ASSETS)

    steel = model.material("steel", rgba=(0.56, 0.58, 0.60, 1.0))
    dark_oxide = model.material("dark_oxide", rgba=(0.17, 0.18, 0.19, 1.0))
    zinc = model.material("zinc", rgba=(0.72, 0.74, 0.78, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.39, 0.41, 0.43, 1.0))
    bronze = model.material("bronze", rgba=(0.63, 0.47, 0.24, 1.0))

    base = model.part("base_frame")
    base.visual(
        mesh_from_cadquery(_make_base_frame(), "base_frame.obj", assets=ASSETS),
        material=steel,
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_X, BASE_Y, 0.082)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
    )

    left_cover = model.part("left_service_cover")
    left_cover.visual(
        mesh_from_cadquery(_make_service_cover(-1.0), "left_service_cover.obj", assets=ASSETS),
        material=cover_gray,
        name="left_cover_shell",
    )
    left_cover.inertial = Inertial.from_geometry(Box((SERVICE_COVER_X, SERVICE_COVER_Y, 0.006)), mass=0.08)

    right_cover = model.part("right_service_cover")
    right_cover.visual(
        mesh_from_cadquery(_make_service_cover(1.0), "right_service_cover.obj", assets=ASSETS),
        material=cover_gray,
        name="right_cover_shell",
    )
    right_cover.inertial = Inertial.from_geometry(Box((SERVICE_COVER_X, SERVICE_COVER_Y, 0.006)), mass=0.08)

    front_bushing = model.part("front_pitch_bushing")
    front_bushing.visual(
        mesh_from_cadquery(_make_pitch_bushing(), "front_pitch_bushing.obj", assets=ASSETS),
        material=bronze,
        name="front_pitch_bushing_shell",
    )
    front_bushing.inertial = Inertial.from_geometry(
        Cylinder(radius=PITCH_BUSHING_OR, length=PITCH_BUSHING_LEN),
        mass=0.04,
    )

    rear_bushing = model.part("rear_pitch_bushing")
    rear_bushing.visual(
        mesh_from_cadquery(_make_pitch_bushing(), "rear_pitch_bushing.obj", assets=ASSETS),
        material=bronze,
        name="rear_pitch_bushing_shell",
    )
    rear_bushing.inertial = Inertial.from_geometry(
        Cylinder(radius=PITCH_BUSHING_OR, length=PITCH_BUSHING_LEN),
        mass=0.04,
    )

    front_cap = model.part("front_trunnion_cap")
    front_cap.visual(
        mesh_from_cadquery(_make_trunnion_cap(1.0), "front_trunnion_cap.obj", assets=ASSETS),
        material=zinc,
        name="front_cap_shell",
    )
    front_cap.inertial = Inertial.from_geometry(Box((0.058, CAP_T, 0.038)), mass=0.07)

    rear_cap = model.part("rear_trunnion_cap")
    rear_cap.visual(
        mesh_from_cadquery(_make_trunnion_cap(-1.0), "rear_trunnion_cap.obj", assets=ASSETS),
        material=zinc,
        name="rear_cap_shell",
    )
    rear_cap.inertial = Inertial.from_geometry(Box((0.058, CAP_T, 0.038)), mass=0.07)

    lower_boot_ring = model.part("lower_boot_ring")
    lower_boot_ring.visual(
        mesh_from_cadquery(_make_lower_boot_ring(), "lower_boot_ring.obj", assets=ASSETS),
        material=dark_oxide,
        name="lower_boot_ring_shell",
    )
    lower_boot_ring.inertial = Inertial.from_geometry(Cylinder(radius=LOWER_BOOT_OR, length=0.004), mass=0.10)

    upper_boot_clamp = model.part("upper_boot_clamp")
    upper_boot_clamp.visual(
        mesh_from_cadquery(_make_upper_boot_clamp(), "upper_boot_clamp.obj", assets=ASSETS),
        material=zinc,
        name="upper_boot_clamp_shell",
    )
    upper_boot_clamp.inertial = Inertial.from_geometry(Cylinder(radius=UPPER_BOOT_OR, length=0.003), mass=0.08)

    outer_yoke = model.part("outer_pitch_yoke")
    outer_yoke.visual(
        mesh_from_cadquery(_make_outer_yoke(), "outer_pitch_yoke.obj", assets=ASSETS),
        material=dark_oxide,
        name="outer_yoke_shell",
    )
    outer_yoke.inertial = Inertial.from_geometry(Box((0.110, 0.174, 0.082)), mass=1.05)

    left_roll_bearing = model.part("left_roll_bearing")
    left_roll_bearing.visual(
        mesh_from_cadquery(_make_roll_bearing(), "left_roll_bearing.obj", assets=ASSETS),
        material=bronze,
        name="left_roll_bearing_shell",
    )
    left_roll_bearing.inertial = Inertial.from_geometry(
        Cylinder(radius=ROLL_BUSHING_OR, length=ROLL_BUSHING_LEN),
        mass=0.03,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    right_roll_bearing = model.part("right_roll_bearing")
    right_roll_bearing.visual(
        mesh_from_cadquery(_make_roll_bearing(), "right_roll_bearing.obj", assets=ASSETS),
        material=bronze,
        name="right_roll_bearing_shell",
    )
    right_roll_bearing.inertial = Inertial.from_geometry(
        Cylinder(radius=ROLL_BUSHING_OR, length=ROLL_BUSHING_LEN),
        mass=0.03,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    inner_carrier = model.part("inner_roll_carrier")
    inner_carrier.visual(
        mesh_from_cadquery(_make_inner_carrier(), "inner_roll_carrier.obj", assets=ASSETS),
        material=steel,
        name="inner_carrier_shell",
    )
    inner_carrier.inertial = Inertial.from_geometry(Box((0.102, 0.094, 0.084)), mass=0.72)

    shaft = model.part("control_shaft")
    shaft.visual(
        mesh_from_cadquery(_make_control_shaft(), "control_shaft.obj", assets=ASSETS),
        material=steel,
        name="control_shaft_shell",
    )
    shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.012, length=0.234),
        mass=0.55,
    )

    model.articulation(
        "base_to_left_service_cover",
        ArticulationType.FIXED,
        parent=base,
        child=left_cover,
        origin=Origin(xyz=(-SERVICE_COVER_POS_X, 0.0, BASE_T)),
    )
    model.articulation(
        "base_to_right_service_cover",
        ArticulationType.FIXED,
        parent=base,
        child=right_cover,
        origin=Origin(xyz=(SERVICE_COVER_POS_X, 0.0, BASE_T)),
    )
    model.articulation(
        "base_to_front_pitch_bushing",
        ArticulationType.FIXED,
        parent=base,
        child=front_bushing,
        origin=Origin(xyz=(0.0, PITCH_BRACKET_Y, PIVOT_Z)),
    )
    model.articulation(
        "base_to_rear_pitch_bushing",
        ArticulationType.FIXED,
        parent=base,
        child=rear_bushing,
        origin=Origin(xyz=(0.0, -PITCH_BRACKET_Y, PIVOT_Z)),
    )
    model.articulation(
        "base_to_front_trunnion_cap",
        ArticulationType.FIXED,
        parent=base,
        child=front_cap,
        origin=Origin(xyz=(0.0, PITCH_BRACKET_Y + PITCH_BRACKET_T / 2 + CAP_T / 2, PIVOT_Z)),
    )
    model.articulation(
        "base_to_rear_trunnion_cap",
        ArticulationType.FIXED,
        parent=base,
        child=rear_cap,
        origin=Origin(xyz=(0.0, -(PITCH_BRACKET_Y + PITCH_BRACKET_T / 2 + CAP_T / 2), PIVOT_Z)),
    )
    model.articulation(
        "base_to_lower_boot_ring",
        ArticulationType.FIXED,
        parent=base,
        child=lower_boot_ring,
        origin=Origin(xyz=(0.0, 0.0, BASE_T + 0.002)),
    )
    model.articulation(
        "base_to_upper_boot_clamp",
        ArticulationType.FIXED,
        parent=base,
        child=upper_boot_clamp,
        origin=Origin(xyz=(0.0, 0.0, BASE_T + 0.0055)),
    )
    model.articulation(
        "base_to_outer_pitch_yoke",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_yoke,
        origin=Origin(xyz=(0.0, 0.0, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.5, lower=-0.42, upper=0.42),
    )
    model.articulation(
        "outer_pitch_yoke_to_left_roll_bearing",
        ArticulationType.FIXED,
        parent=outer_yoke,
        child=left_roll_bearing,
        origin=Origin(xyz=(-OUTER_ARM_OFFSET_X, 0.0, 0.0)),
    )
    model.articulation(
        "outer_pitch_yoke_to_right_roll_bearing",
        ArticulationType.FIXED,
        parent=outer_yoke,
        child=right_roll_bearing,
        origin=Origin(xyz=(OUTER_ARM_OFFSET_X, 0.0, 0.0)),
    )
    model.articulation(
        "outer_pitch_yoke_to_inner_roll_carrier",
        ArticulationType.REVOLUTE,
        parent=outer_yoke,
        child=inner_carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.8, lower=-0.38, upper=0.38),
    )
    model.articulation(
        "inner_roll_carrier_to_control_shaft",
        ArticulationType.FIXED,
        parent=inner_carrier,
        child=shaft,
        origin=Origin(),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base_frame")
    left_cover = object_model.get_part("left_service_cover")
    right_cover = object_model.get_part("right_service_cover")
    front_bushing = object_model.get_part("front_pitch_bushing")
    rear_bushing = object_model.get_part("rear_pitch_bushing")
    front_cap = object_model.get_part("front_trunnion_cap")
    rear_cap = object_model.get_part("rear_trunnion_cap")
    lower_boot_ring = object_model.get_part("lower_boot_ring")
    upper_boot_clamp = object_model.get_part("upper_boot_clamp")
    outer_yoke = object_model.get_part("outer_pitch_yoke")
    left_roll_bearing = object_model.get_part("left_roll_bearing")
    right_roll_bearing = object_model.get_part("right_roll_bearing")
    inner_carrier = object_model.get_part("inner_roll_carrier")
    shaft = object_model.get_part("control_shaft")
    pitch_joint = object_model.get_articulation("base_to_outer_pitch_yoke")
    roll_joint = object_model.get_articulation("outer_pitch_yoke_to_inner_roll_carrier")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(base, left_cover, reason="service cover sits within a shallow machined rabbet in the base plate")
    ctx.allow_overlap(base, right_cover, reason="service cover sits within a shallow machined rabbet in the base plate")
    ctx.allow_overlap(base, lower_boot_ring, reason="boot retention ring nests into the counterbored opening land")
    ctx.allow_overlap(base, front_bushing, reason="bronze pitch bushing is press-fit into the base bracket bore")
    ctx.allow_overlap(base, rear_bushing, reason="bronze pitch bushing is press-fit into the base bracket bore")
    ctx.allow_overlap(front_bushing, outer_yoke, reason="pitch trunnion journal runs concentrically inside the front sleeve bearing")
    ctx.allow_overlap(rear_bushing, outer_yoke, reason="pitch trunnion journal runs concentrically inside the rear sleeve bearing")
    ctx.allow_overlap(left_roll_bearing, outer_yoke, reason="left roll bearing shell is retained in the yoke cheek bore")
    ctx.allow_overlap(right_roll_bearing, outer_yoke, reason="right roll bearing shell is retained in the yoke cheek bore")
    ctx.allow_overlap(inner_carrier, left_roll_bearing, reason="left roll journal nests inside the retained sleeve bearing")
    ctx.allow_overlap(inner_carrier, right_roll_bearing, reason="right roll journal nests inside the retained sleeve bearing")
    ctx.allow_overlap(shaft, inner_carrier, reason="split clamp collar grips the central shaft through a tightened bore")
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(left_cover, base, name="left_service_cover_seated")
    ctx.expect_contact(right_cover, base, name="right_service_cover_seated")
    ctx.expect_contact(front_bushing, base, name="front_pitch_bushing_seated")
    ctx.expect_contact(rear_bushing, base, name="rear_pitch_bushing_seated")
    ctx.expect_contact(front_cap, base, name="front_trunnion_cap_fastened")
    ctx.expect_contact(rear_cap, base, name="rear_trunnion_cap_fastened")
    ctx.expect_contact(lower_boot_ring, base, contact_tol=0.001, name="lower_boot_ring_clamped")
    ctx.expect_contact(upper_boot_clamp, lower_boot_ring, contact_tol=0.002, name="upper_boot_clamp_stacked")
    ctx.expect_origin_distance(front_bushing, outer_yoke, axes="xz", max_dist=0.001, name="front_pitch_bearing_loaded")
    ctx.expect_origin_distance(rear_bushing, outer_yoke, axes="xz", max_dist=0.001, name="rear_pitch_bearing_loaded")
    ctx.expect_contact(left_roll_bearing, outer_yoke, name="left_roll_bearing_seated")
    ctx.expect_contact(right_roll_bearing, outer_yoke, name="right_roll_bearing_seated")
    ctx.expect_origin_distance(inner_carrier, left_roll_bearing, axes="yz", max_dist=0.001, name="left_roll_interface_supported")
    ctx.expect_origin_distance(inner_carrier, right_roll_bearing, axes="yz", max_dist=0.001, name="right_roll_interface_supported")
    ctx.expect_contact(shaft, inner_carrier, name="shaft_clamped_in_inner_carrier")

    ctx.expect_overlap(lower_boot_ring, base, axes="xy", min_overlap=0.105, name="boot_ring_centered_on_base")
    ctx.expect_overlap(upper_boot_clamp, lower_boot_ring, axes="xy", min_overlap=0.090, name="boot_clamp_registered_to_lower_ring")
    ctx.expect_overlap(outer_yoke, base, axes="xz", min_overlap=0.028, name="outer_yoke_registered_in_support_frame")
    ctx.expect_overlap(inner_carrier, outer_yoke, axes="yz", min_overlap=0.040, name="inner_carrier_nested_in_outer_yoke")
    ctx.expect_overlap(shaft, lower_boot_ring, axes="xy", min_overlap=0.020, name="shaft_centered_over_boot_opening")
    ctx.expect_origin_distance(inner_carrier, outer_yoke, axes="xy", max_dist=0.001, name="gimbal_axes_concentric_at_rest")
    ctx.expect_origin_distance(shaft, inner_carrier, axes="xy", max_dist=0.001, name="shaft_concentric_with_roll_carrier")

    with ctx.pose({pitch_joint: 0.34}):
        ctx.expect_origin_distance(front_bushing, outer_yoke, axes="xz", max_dist=0.001, name="pitch_pose_front_pitch_bearing_loaded")
        ctx.expect_origin_distance(rear_bushing, outer_yoke, axes="xz", max_dist=0.001, name="pitch_pose_rear_pitch_bearing_loaded")
        ctx.expect_overlap(inner_carrier, left_roll_bearing, axes="yz", min_overlap=0.018, name="pitch_pose_left_roll_bearing_loaded")
        ctx.expect_overlap(inner_carrier, right_roll_bearing, axes="yz", min_overlap=0.018, name="pitch_pose_right_roll_bearing_loaded")
        ctx.expect_overlap(shaft, lower_boot_ring, axes="xy", min_overlap=0.014, name="pitch_pose_shaft_stays_over_boot_ring")

    with ctx.pose({roll_joint: -0.30}):
        ctx.expect_origin_distance(inner_carrier, left_roll_bearing, axes="yz", max_dist=0.001, name="roll_pose_left_roll_bearing_loaded")
        ctx.expect_origin_distance(inner_carrier, right_roll_bearing, axes="yz", max_dist=0.001, name="roll_pose_right_roll_bearing_loaded")
        ctx.expect_overlap(shaft, lower_boot_ring, axes="xy", min_overlap=0.014, name="roll_pose_shaft_stays_over_boot_ring")

    with ctx.pose({pitch_joint: -0.22, roll_joint: 0.26}):
        ctx.expect_origin_distance(front_bushing, outer_yoke, axes="xz", max_dist=0.001, name="combined_pose_front_pitch_bearing_loaded")
        ctx.expect_origin_distance(rear_bushing, outer_yoke, axes="xz", max_dist=0.001, name="combined_pose_rear_pitch_bearing_loaded")
        ctx.expect_overlap(inner_carrier, left_roll_bearing, axes="yz", min_overlap=0.020, name="combined_pose_left_roll_bearing_loaded")
        ctx.expect_overlap(inner_carrier, right_roll_bearing, axes="yz", min_overlap=0.020, name="combined_pose_right_roll_bearing_loaded")
        ctx.expect_overlap(shaft, lower_boot_ring, axes="xy", min_overlap=0.011, name="combined_pose_shaft_remains_within_base_opening_footprint")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
