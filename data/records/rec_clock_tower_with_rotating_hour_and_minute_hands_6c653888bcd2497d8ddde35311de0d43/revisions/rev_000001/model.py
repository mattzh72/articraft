from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


PLINTH_SIZE = 2.8
PLINTH_HEIGHT = 0.80
SHAFT_OUTER = 1.55
SHAFT_INNER = 1.31
SHAFT_HEIGHT = 11.80
BASE_COLLAR_HEIGHT = 0.18
CROWN_HEIGHT = 0.22
CLOCK_BOX_SIZE = 1.44
CLOCK_BOX_BODY_SIZE = 1.34
CLOCK_BOX_DEPTH = 0.42
CLOCK_CENTER_Z = 10.60


def _square_profile(size: float) -> list[tuple[float, float]]:
    half = size * 0.5
    return [
        (-half, -half),
        (half, -half),
        (half, half),
        (-half, half),
    ]


def _square_ring_mesh(name: str, outer_size: float, inner_size: float, height: float):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _square_profile(outer_size),
            [list(reversed(_square_profile(inner_size)))],
            height=height,
            center=False,
        ),
        name,
    )


def _face_ring_mesh(name: str, outer_size: float, inner_size: float, depth: float):
    geom = ExtrudeWithHolesGeometry(
        _square_profile(outer_size),
        [list(reversed(_square_profile(inner_size)))],
        height=depth,
        center=False,
    )
    geom.rotate_x(pi / 2.0)
    geom.translate(0.0, depth, 0.0)
    return mesh_from_geometry(geom, name)


def _hand_mesh(
    name: str,
    *,
    thickness: float,
    hub_radius: float,
    blade_width: float,
    blade_length: float,
    tip_width: float,
    tip_length: float,
    tail_width: float,
    tail_length: float,
    angle: float,
    hub_overlap: float,
    tip_overlap: float,
):
    geom = CylinderGeometry(hub_radius, thickness, radial_segments=28).rotate_x(pi / 2.0)
    geom.translate(0.0, thickness * 0.5, 0.0)
    geom.merge(
        BoxGeometry((blade_width, thickness, blade_length)).translate(
            0.0,
            thickness * 0.5,
            blade_length * 0.5 - hub_overlap,
        )
    )
    geom.merge(
        BoxGeometry((tip_width, thickness, tip_length)).translate(
            0.0,
            thickness * 0.5,
            blade_length - tip_overlap + tip_length * 0.5,
        )
    )
    geom.merge(
        BoxGeometry((tail_width, thickness, tail_length)).translate(
            0.0,
            thickness * 0.5,
            -(tail_length * 0.5 - hub_overlap * 0.5),
        )
    )
    geom.rotate_y(angle)
    return mesh_from_geometry(geom, name)


def _add_clock_box_visuals(part, *, steel_dark, steel_light, dial_light, clock_black) -> None:
    bezel_mesh = _face_ring_mesh("clock_face_bezel", CLOCK_BOX_SIZE, 1.12, 0.06)
    part.visual(
        Box((CLOCK_BOX_BODY_SIZE, 0.28, CLOCK_BOX_BODY_SIZE)),
        origin=Origin(xyz=(0.0, 0.14, 0.0)),
        material=steel_dark,
        name="housing_body",
    )
    part.visual(
        bezel_mesh,
        origin=Origin(xyz=(0.0, 0.280, 0.0)),
        material=steel_light,
        name="front_frame",
    )
    part.visual(
        Cylinder(radius=0.562, length=0.010),
        origin=Origin(xyz=(0.0, 0.305, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dial_light,
        name="dial_face",
    )
    part.visual(
        Cylinder(radius=0.10, length=0.006),
        origin=Origin(xyz=(0.0, 0.313, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_light,
        name="hour_pivot_pad",
    )
    part.visual(
        Cylinder(radius=0.055, length=0.004),
        origin=Origin(xyz=(0.0, 0.318, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel_light,
        name="minute_pivot_pad",
    )
    marker_radius = 0.42
    for index in range(12):
        angle = (2.0 * pi * index) / 12.0
        part.visual(
            Box((0.030, 0.010, 0.110 if index % 3 == 0 else 0.075)),
            origin=Origin(
                xyz=(marker_radius * sin(angle), 0.309, marker_radius * cos(angle)),
                rpy=(0.0, angle, 0.0),
            ),
            material=clock_black,
            name=f"hour_marker_{index:02d}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="civic_clock_tower")

    concrete = model.material("concrete", rgba=(0.71, 0.72, 0.73, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.19, 0.22, 0.26, 1.0))
    steel_light = model.material("steel_light", rgba=(0.48, 0.51, 0.56, 1.0))
    dial_light = model.material("dial_light", rgba=(0.93, 0.94, 0.90, 1.0))
    clock_black = model.material("clock_black", rgba=(0.09, 0.10, 0.12, 1.0))

    shaft_shell_mesh = _square_ring_mesh("clock_tower_shaft_shell", SHAFT_OUTER, SHAFT_INNER, SHAFT_HEIGHT)
    shaft_base_collar_mesh = _square_ring_mesh(
        "clock_tower_shaft_base_collar",
        SHAFT_OUTER + 0.12,
        SHAFT_INNER,
        BASE_COLLAR_HEIGHT,
    )
    shaft_crown_mesh = _square_ring_mesh(
        "clock_tower_shaft_crown",
        SHAFT_OUTER + 0.16,
        SHAFT_INNER,
        CROWN_HEIGHT,
    )
    hour_hand_mesh = _hand_mesh(
        "clock_hour_hand",
        thickness=0.008,
        hub_radius=0.082,
        blade_width=0.092,
        blade_length=0.18,
        tip_width=0.044,
        tip_length=0.15,
        tail_width=0.034,
        tail_length=0.10,
        angle=-0.96,
        hub_overlap=0.035,
        tip_overlap=0.026,
    )
    minute_hand_mesh = _hand_mesh(
        "clock_minute_hand",
        thickness=0.006,
        hub_radius=0.043,
        blade_width=0.046,
        blade_length=0.24,
        tip_width=0.024,
        tip_length=0.19,
        tail_width=0.018,
        tail_length=0.13,
        angle=pi / 3.0,
        hub_overlap=0.028,
        tip_overlap=0.022,
    )
    plinth = model.part("plinth")
    plinth.visual(
        Box((PLINTH_SIZE, PLINTH_SIZE, 0.68)),
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        material=concrete,
        name="plinth_base",
    )
    plinth.visual(
        Box((2.20, 2.20, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.74)),
        material=concrete,
        name="plinth_cap",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((PLINTH_SIZE, PLINTH_SIZE, PLINTH_HEIGHT)),
        mass=24000.0,
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT * 0.5)),
    )

    shaft = model.part("shaft")
    shaft.visual(shaft_shell_mesh, material=steel_dark, name="tube_shell")
    shaft.visual(shaft_base_collar_mesh, material=steel_light, name="base_collar")
    shaft.visual(
        shaft_crown_mesh,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_HEIGHT - CROWN_HEIGHT)),
        material=steel_light,
        name="crown_band",
    )
    shaft.inertial = Inertial.from_geometry(
        Box((SHAFT_OUTER + 0.16, SHAFT_OUTER + 0.16, SHAFT_HEIGHT)),
        mass=9500.0,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_HEIGHT * 0.5)),
    )

    model.articulation(
        "plinth_to_shaft",
        ArticulationType.FIXED,
        parent=plinth,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, PLINTH_HEIGHT)),
    )

    face_specs = (
        ("north_clock", (0.0, SHAFT_OUTER * 0.5, CLOCK_CENTER_Z), (0.0, 0.0, 0.0)),
        ("east_clock", (SHAFT_OUTER * 0.5, 0.0, CLOCK_CENTER_Z), (0.0, 0.0, -pi / 2.0)),
        ("south_clock", (0.0, -SHAFT_OUTER * 0.5, CLOCK_CENTER_Z), (0.0, 0.0, pi)),
        ("west_clock", (-SHAFT_OUTER * 0.5, 0.0, CLOCK_CENTER_Z), (0.0, 0.0, pi / 2.0)),
    )

    for clock_name, xyz, rpy in face_specs:
        box = model.part(clock_name)
        box.inertial = Inertial.from_geometry(
            Box((CLOCK_BOX_SIZE, CLOCK_BOX_DEPTH, CLOCK_BOX_SIZE)),
            mass=520.0,
            origin=Origin(xyz=(0.0, CLOCK_BOX_DEPTH * 0.5, 0.0)),
        )
        _add_clock_box_visuals(
            box,
            steel_dark=steel_dark,
            steel_light=steel_light,
            dial_light=dial_light,
            clock_black=clock_black,
        )
        model.articulation(
            f"shaft_to_{clock_name}",
            ArticulationType.FIXED,
            parent=shaft,
            child=box,
            origin=Origin(xyz=xyz, rpy=rpy),
        )

        hour_hand = model.part(f"{clock_name}_hour_hand")
        hour_hand.visual(hour_hand_mesh, material=clock_black, name="hand_body")
        hour_hand.inertial = Inertial.from_geometry(
            Box((0.12, 0.008, 0.48)),
            mass=8.0,
            origin=Origin(xyz=(0.0, 0.004, 0.09)),
        )
        model.articulation(
            f"{clock_name}_hour_joint",
            ArticulationType.REVOLUTE,
            parent=box,
            child=hour_hand,
            origin=Origin(xyz=(0.0, 0.312, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.25, lower=-2.0 * pi, upper=2.0 * pi),
        )

        minute_hand = model.part(f"{clock_name}_minute_hand")
        minute_hand.visual(minute_hand_mesh, material=clock_black, name="hand_body")
        minute_hand.inertial = Inertial.from_geometry(
            Box((0.07, 0.006, 0.64)),
            mass=5.0,
            origin=Origin(xyz=(0.0, 0.003, 0.14)),
        )
        model.articulation(
            f"{clock_name}_minute_joint",
            ArticulationType.REVOLUTE,
            parent=box,
            child=minute_hand,
            origin=Origin(xyz=(0.0, 0.320, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=0.25, lower=-2.0 * pi, upper=2.0 * pi),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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
    ctx.fail_if_parts_overlap_in_current_pose()

    plinth = object_model.get_part("plinth")
    shaft = object_model.get_part("shaft")
    ctx.expect_contact(shaft, plinth, name="shaft_bears_on_plinth")

    face_axes = {
        "north_clock": (1, 1.0, (0, 2)),
        "east_clock": (0, 1.0, (1, 2)),
        "south_clock": (1, -1.0, (0, 2)),
        "west_clock": (0, -1.0, (1, 2)),
    }

    for face_name, (axis_index, outward_sign, tangent_axes) in face_axes.items():
        box = object_model.get_part(face_name)
        hour_hand = object_model.get_part(f"{face_name}_hour_hand")
        minute_hand = object_model.get_part(f"{face_name}_minute_hand")
        hour_joint = object_model.get_articulation(f"{face_name}_hour_joint")
        minute_joint = object_model.get_articulation(f"{face_name}_minute_joint")

        ctx.expect_contact(box, shaft, name=f"{face_name}_box_contacts_shaft")
        ctx.expect_contact(hour_hand, box, name=f"{face_name}_hour_mounts_to_face")
        ctx.expect_contact(minute_hand, hour_hand, name=f"{face_name}_minute_stacks_on_hour")
        ctx.check(
            f"{face_name}_hour_axis",
            hour_joint.articulation_type == ArticulationType.REVOLUTE and hour_joint.axis == (0.0, 1.0, 0.0),
            details=f"type={hour_joint.articulation_type}, axis={hour_joint.axis}",
        )
        ctx.check(
            f"{face_name}_minute_axis",
            minute_joint.articulation_type == ArticulationType.REVOLUTE and minute_joint.axis == (0.0, 1.0, 0.0),
            details=f"type={minute_joint.articulation_type}, axis={minute_joint.axis}",
        )

        box_pos = ctx.part_world_position(box)
        hour_pos = ctx.part_world_position(hour_hand)
        minute_pos = ctx.part_world_position(minute_hand)
        assert box_pos is not None and hour_pos is not None and minute_pos is not None

        box_out = box_pos[axis_index] * outward_sign
        hour_out = hour_pos[axis_index] * outward_sign
        minute_out = minute_pos[axis_index] * outward_sign
        ctx.check(
            f"{face_name}_hands_project_forward",
            hour_out > box_out + 0.30 and minute_out > hour_out + 0.002,
            details=f"box={box_out:.4f}, hour={hour_out:.4f}, minute={minute_out:.4f}",
        )

        tangent_ok = all(
            abs(box_pos[idx] - hour_pos[idx]) <= 1e-6 and abs(box_pos[idx] - minute_pos[idx]) <= 1e-6
            for idx in tangent_axes
        )
        ctx.check(
            f"{face_name}_hands_centered_on_face",
            tangent_ok,
            details=f"box={box_pos}, hour={hour_pos}, minute={minute_pos}",
        )

    north_minute = object_model.get_part("north_clock_minute_hand")
    north_minute_joint = object_model.get_articulation("north_clock_minute_joint")
    rest_aabb = ctx.part_element_world_aabb(north_minute, elem="hand_body")
    assert rest_aabb is not None
    with ctx.pose({north_minute_joint: pi / 6.0}):
        turned_aabb = ctx.part_element_world_aabb(north_minute, elem="hand_body")
        assert turned_aabb is not None
        ctx.check(
            "north_minute_joint_rotates_hand",
            turned_aabb[1][0] > rest_aabb[1][0] + 0.05 and turned_aabb[0][2] < rest_aabb[0][2] - 0.04,
            details=f"rest={rest_aabb}, turned={turned_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
