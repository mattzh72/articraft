from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_GUIDE_INNER_R = 0.0155
BASE_GUIDE_OUTER_R = 0.0240
REAR_ROD_R = 0.0140

MID_GUIDE_INNER_R = 0.0112
MID_GUIDE_OUTER_R = 0.0180
MID_ROD_R = 0.0100

FRONT_GUIDE_INNER_R = 0.0072
FRONT_GUIDE_OUTER_R = 0.0130
FRONT_ROD_R = 0.0060

REAR_TRAVEL = 0.080
MID_TRAVEL = 0.070
FRONT_TRAVEL = 0.060


def x_cylinder(radius: float, length: float, x0: float = 0.0) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((x0, 0.0, 0.0))


def x_tube(
    outer_r: float,
    inner_r: float,
    length: float,
    x0: float = 0.0,
) -> cq.Workplane:
    outer = x_cylinder(outer_r, length, x0)
    inner = x_cylinder(inner_r, length + 0.002, x0 - 0.001)
    return outer.cut(inner)


def x_box(length: float, width: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(center)


def y_cylinder(
    radius: float,
    length: float,
    *,
    x: float,
    z: float,
    y0: float | None = None,
) -> cq.Workplane:
    start_y = -length / 2.0 if y0 is None else y0
    return cq.Workplane("XZ").circle(radius).extrude(length).translate((x, start_y, z))


def add_shapes(first: cq.Workplane, *rest: cq.Workplane) -> cq.Workplane:
    shape = first
    for item in rest:
        shape = shape.union(item)
    return shape


def clamp_ring(
    *,
    x0: float,
    length: float,
    base_outer_r: float,
    clamp_outer_r: float,
    ear_width: float,
    ear_height: float,
    screw_r: float,
    head_r: float,
    screw_length: float,
) -> cq.Workplane:
    ring = x_tube(clamp_outer_r, max(base_outer_r - 0.001, 0.001), length, x0)
    x_center = x0 + length / 2.0
    ear_z = base_outer_r + ear_height / 2.0 - 0.001

    top_ear = x_box(length, ear_width, ear_height, (x_center, 0.0, ear_z))
    bottom_ear = x_box(length, ear_width, ear_height, (x_center, 0.0, -ear_z))

    top_shank = y_cylinder(screw_r, screw_length, x=x_center, z=ear_z)
    bottom_shank = y_cylinder(screw_r, screw_length, x=x_center, z=-ear_z)

    head_len = 0.004
    top_head = y_cylinder(head_r, head_len, x=x_center, z=ear_z, y0=screw_length / 2.0 - head_len)
    bottom_head = y_cylinder(head_r, head_len, x=x_center, z=-ear_z, y0=screw_length / 2.0 - head_len)

    return add_shapes(ring, top_ear, bottom_ear, top_shank, bottom_shank, top_head, bottom_head)


def build_base_body() -> tuple[cq.Workplane, cq.Workplane]:
    base_block = x_box(0.180, 0.120, 0.070, (-0.035, 0.0, -0.020))
    foot = x_box(0.220, 0.092, 0.012, (-0.035, 0.0, -0.061))
    front_saddle = x_box(0.056, 0.090, 0.050, (0.054, 0.0, -0.020))
    top_pad = x_box(0.070, 0.078, 0.016, (0.010, 0.0, 0.006))

    outer_guide = x_tube(BASE_GUIDE_OUTER_R, BASE_GUIDE_INNER_R, 0.208, -0.040)
    rear_ring = x_cylinder(BASE_GUIDE_OUTER_R + 0.004, 0.014, -0.054)
    clamp = clamp_ring(
        x0=0.134,
        length=0.014,
        base_outer_r=BASE_GUIDE_OUTER_R,
        clamp_outer_r=BASE_GUIDE_OUTER_R + 0.005,
        ear_width=0.040,
        ear_height=0.010,
        screw_r=0.0025,
        head_r=0.0045,
        screw_length=0.044,
    )

    body = add_shapes(base_block, foot, front_saddle, top_pad, outer_guide, rear_ring, clamp)
    front_cap = x_tube(BASE_GUIDE_OUTER_R + 0.003, BASE_GUIDE_INNER_R, 0.012, 0.168)
    return body, front_cap


def build_rear_stage() -> tuple[cq.Workplane, cq.Workplane, cq.Workplane]:
    rod = x_cylinder(REAR_ROD_R, 0.120, 0.010)
    shoulder = x_cylinder(REAR_ROD_R + 0.002, 0.018, 0.130)
    guide_mount = x_cylinder(MID_GUIDE_OUTER_R, 0.032, 0.148)
    mid_guide = x_tube(MID_GUIDE_OUTER_R, MID_GUIDE_INNER_R, 0.090, 0.180)
    rear_guide_ring = x_tube(MID_GUIDE_OUTER_R + 0.002, MID_GUIDE_OUTER_R - 0.001, 0.010, 0.176)
    clamp = clamp_ring(
        x0=0.228,
        length=0.014,
        base_outer_r=MID_GUIDE_OUTER_R,
        clamp_outer_r=MID_GUIDE_OUTER_R + 0.005,
        ear_width=0.034,
        ear_height=0.008,
        screw_r=0.0022,
        head_r=0.0040,
        screw_length=0.038,
    )

    body = add_shapes(rod, shoulder, guide_mount, mid_guide, rear_guide_ring, clamp)
    stop_collar = x_tube(BASE_GUIDE_INNER_R + 0.0045, REAR_ROD_R, 0.010, 0.000)
    front_cap = x_tube(MID_GUIDE_OUTER_R + 0.0025, MID_GUIDE_INNER_R, 0.010, 0.270)
    return body, stop_collar, front_cap


def build_middle_stage() -> tuple[cq.Workplane, cq.Workplane, cq.Workplane]:
    rod = x_cylinder(MID_ROD_R, 0.100, 0.010)
    shoulder = x_cylinder(MID_ROD_R + 0.0018, 0.016, 0.110)
    guide_mount = x_cylinder(FRONT_GUIDE_OUTER_R, 0.028, 0.126)
    front_guide = x_tube(FRONT_GUIDE_OUTER_R, FRONT_GUIDE_INNER_R, 0.050, 0.160)
    rear_guide_ring = x_tube(FRONT_GUIDE_OUTER_R + 0.002, FRONT_GUIDE_OUTER_R - 0.001, 0.010, 0.156)
    clamp = clamp_ring(
        x0=0.182,
        length=0.012,
        base_outer_r=FRONT_GUIDE_OUTER_R,
        clamp_outer_r=FRONT_GUIDE_OUTER_R + 0.004,
        ear_width=0.028,
        ear_height=0.007,
        screw_r=0.0018,
        head_r=0.0035,
        screw_length=0.032,
    )

    body = add_shapes(rod, shoulder, guide_mount, front_guide, rear_guide_ring, clamp)
    stop_collar = x_tube(MID_GUIDE_INNER_R + 0.0038, MID_ROD_R, 0.010, 0.000)
    front_cap = x_tube(FRONT_GUIDE_OUTER_R + 0.002, FRONT_GUIDE_INNER_R, 0.010, 0.210)
    return body, stop_collar, front_cap


def build_front_stage() -> tuple[cq.Workplane, cq.Workplane]:
    rod = x_cylinder(FRONT_ROD_R, 0.112, 0.008)
    nose = x_cylinder(0.0040, 0.040, 0.136)
    tip_shoulder = x_cylinder(0.0050, 0.016, 0.120)
    nose_button = x_cylinder(0.0065, 0.010, 0.176)

    body = add_shapes(rod, tip_shoulder, nose, nose_button)
    stop_collar = x_tube(FRONT_GUIDE_INNER_R + 0.0028, FRONT_ROD_R, 0.008, 0.000)
    return body, stop_collar


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ejector_plunger_stack")

    model.material("base_gray", rgba=(0.31, 0.33, 0.35, 1.0))
    model.material("guide_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("cap_black", rgba=(0.15, 0.16, 0.18, 1.0))
    model.material("rod_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("tip_steel", rgba=(0.70, 0.72, 0.75, 1.0))

    base_body, outer_front_cap = build_base_body()
    rear_body, rear_stop_collar, mid_front_cap = build_rear_stage()
    middle_body, middle_stop_collar, front_front_cap = build_middle_stage()
    front_body, front_stop_collar = build_front_stage()

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(base_body, "base_body"),
        material="base_gray",
        name="base_body",
    )
    base.visual(
        mesh_from_cadquery(outer_front_cap, "outer_front_cap"),
        material="cap_black",
        name="outer_front_cap",
    )

    rear = model.part("rear_stage")
    rear.visual(
        mesh_from_cadquery(rear_body, "rear_stage_body"),
        material="guide_dark",
        name="rear_stage_body",
    )
    rear.visual(
        mesh_from_cadquery(rear_stop_collar, "rear_stop_collar"),
        material="cap_black",
        name="rear_stop_collar",
    )
    rear.visual(
        mesh_from_cadquery(mid_front_cap, "mid_front_cap"),
        material="cap_black",
        name="mid_front_cap",
    )

    middle = model.part("middle_stage")
    middle.visual(
        mesh_from_cadquery(middle_body, "middle_stage_body"),
        material="rod_steel",
        name="middle_stage_body",
    )
    middle.visual(
        mesh_from_cadquery(middle_stop_collar, "middle_stop_collar"),
        material="cap_black",
        name="middle_stop_collar",
    )
    middle.visual(
        mesh_from_cadquery(front_front_cap, "front_front_cap"),
        material="cap_black",
        name="front_front_cap",
    )

    front = model.part("front_stage")
    front.visual(
        mesh_from_cadquery(front_body, "front_stage_body"),
        material="tip_steel",
        name="front_stage_body",
    )
    front.visual(
        mesh_from_cadquery(front_stop_collar, "front_stop_collar"),
        material="cap_black",
        name="front_stop_collar",
    )

    model.articulation(
        "base_to_rear",
        ArticulationType.PRISMATIC,
        parent=base,
        child=rear,
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.25,
            lower=0.0,
            upper=REAR_TRAVEL,
        ),
    )
    model.articulation(
        "rear_to_middle",
        ArticulationType.PRISMATIC,
        parent=rear,
        child=middle,
        origin=Origin(xyz=(0.280, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.28,
            lower=0.0,
            upper=MID_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_front",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=front,
        origin=Origin(xyz=(0.220, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=600.0,
            velocity=0.30,
            lower=0.0,
            upper=FRONT_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    rear = object_model.get_part("rear_stage")
    middle = object_model.get_part("middle_stage")
    front = object_model.get_part("front_stage")

    base_to_rear = object_model.get_articulation("base_to_rear")
    rear_to_middle = object_model.get_articulation("rear_to_middle")
    middle_to_front = object_model.get_articulation("middle_to_front")

    outer_front_cap = base.get_visual("outer_front_cap")
    rear_stop_collar = rear.get_visual("rear_stop_collar")
    mid_front_cap = rear.get_visual("mid_front_cap")
    middle_stop_collar = middle.get_visual("middle_stop_collar")
    front_front_cap = middle.get_visual("front_front_cap")
    front_stop_collar = front.get_visual("front_stop_collar")

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

    for joint_name, joint in (
        ("base_to_rear", base_to_rear),
        ("rear_to_middle", rear_to_middle),
        ("middle_to_front", middle_to_front),
    ):
        ctx.check(
            f"{joint_name}_axis",
            tuple(joint.axis) == (1.0, 0.0, 0.0),
            f"{joint_name} should translate along +X, got axis={joint.axis}",
        )
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name}_limits",
            limits is not None and limits.lower == 0.0 and limits.upper is not None and limits.upper > 0.0,
            f"{joint_name} should have a zeroed retracted stop and positive extension travel",
        )

    with ctx.pose({base_to_rear: 0.0, rear_to_middle: 0.0, middle_to_front: 0.0}):
        ctx.expect_contact(
            base,
            rear,
            elem_a=outer_front_cap,
            elem_b=rear_stop_collar,
            name="rear_stage_retracted_stop_contacts_base_cap",
        )
        ctx.expect_contact(
            rear,
            middle,
            elem_a=mid_front_cap,
            elem_b=middle_stop_collar,
            name="middle_stage_retracted_stop_contacts_mid_cap",
        )
        ctx.expect_contact(
            middle,
            front,
            elem_a=front_front_cap,
            elem_b=front_stop_collar,
            name="front_stage_retracted_stop_contacts_front_cap",
        )

    with ctx.pose(
        {
            base_to_rear: REAR_TRAVEL,
            rear_to_middle: MID_TRAVEL,
            middle_to_front: FRONT_TRAVEL,
        }
    ):
        ctx.expect_gap(
            rear,
            base,
            axis="x",
            positive_elem=rear_stop_collar,
            negative_elem=outer_front_cap,
            min_gap=REAR_TRAVEL - 0.001,
            max_gap=REAR_TRAVEL + 0.001,
            name="rear_stage_extension_gap_matches_travel",
        )
        ctx.expect_gap(
            middle,
            rear,
            axis="x",
            positive_elem=middle_stop_collar,
            negative_elem=mid_front_cap,
            min_gap=MID_TRAVEL - 0.001,
            max_gap=MID_TRAVEL + 0.001,
            name="middle_stage_extension_gap_matches_travel",
        )
        ctx.expect_gap(
            front,
            middle,
            axis="x",
            positive_elem=front_stop_collar,
            negative_elem=front_front_cap,
            min_gap=FRONT_TRAVEL - 0.001,
            max_gap=FRONT_TRAVEL + 0.001,
            name="front_stage_extension_gap_matches_travel",
        )
        ctx.expect_overlap(
            base,
            rear,
            axes="yz",
            elem_a=outer_front_cap,
            elem_b=rear_stop_collar,
            min_overlap=0.020,
            name="rear_stage_stays_coaxial_with_base_guide",
        )
        ctx.expect_overlap(
            rear,
            middle,
            axes="yz",
            elem_a=mid_front_cap,
            elem_b=middle_stop_collar,
            min_overlap=0.014,
            name="middle_stage_stays_coaxial_with_mid_guide",
        )
        ctx.expect_overlap(
            middle,
            front,
            axes="yz",
            elem_a=front_front_cap,
            elem_b=front_stop_collar,
            min_overlap=0.009,
            name="front_stage_stays_coaxial_with_front_guide",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
