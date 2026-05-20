from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)


PALM_LENGTH = 0.120
PALM_WIDTH = 0.148
PALM_THICKNESS = 0.012
PALM_FRONT_X = PALM_LENGTH * 0.5
FLEX_LIMIT = math.radians(80.0)


def _y_axis_cylinder(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(math.pi * 0.5, 0.0, 0.0))


def _add_link_geometry(
    part,
    *,
    mesh_name: str,
    root_radius: float,
    span_to_next_joint: float | None,
    tip_extension: float,
    body_width: float,
    body_height: float,
    main_material,
    pin_material,
    pad_material,
    distal_radius: float | None = None,
    add_tip_pad: bool = False,
) -> None:
    barrel_geom, barrel_origin = _y_axis_cylinder(root_radius, body_width)
    part.visual(barrel_geom, origin=barrel_origin, material=pin_material, name="root_barrel")

    if span_to_next_joint is None:
        body_start = root_radius
        body_length = max(tip_extension - root_radius, 0.004)
        tip_anchor = tip_extension
    else:
        assert distal_radius is not None
        body_start = root_radius
        body_length = max(span_to_next_joint - root_radius - distal_radius, 0.004)
        tip_anchor = span_to_next_joint - distal_radius

    link_profile = [
        (0.0, -body_width * 0.24),
        (body_length * 0.10, -body_width * 0.42),
        (body_length * 0.42, -body_width * 0.40),
        (body_length * 0.78, -body_width * 0.33),
        (body_length, -body_width * 0.28),
        (body_length, body_width * 0.28),
        (body_length * 0.78, body_width * 0.33),
        (body_length * 0.42, body_width * 0.40),
        (body_length * 0.10, body_width * 0.42),
        (0.0, body_width * 0.24),
    ]
    link_body_mesh = mesh_from_geometry(
        ExtrudeGeometry(link_profile, body_height, center=True),
        ASSETS.mesh_path(f"{mesh_name}.obj"),
    )
    part.visual(
        link_body_mesh,
        origin=Origin(xyz=(body_start, 0.0, 0.0)),
        material=main_material,
        name="link_body",
    )
    part.visual(
        Box((body_length * 0.68, body_width * 0.54, body_height * 0.42)),
        origin=Origin(
            xyz=(
                body_start + body_length * 0.54,
                0.0,
                body_height * 0.44,
            )
        ),
        material=main_material,
        name="link_spine",
    )

    if add_tip_pad:
        pad_length = max(tip_extension * 0.32, 0.006)
        part.visual(
            Box((pad_length, body_width * 0.74, body_height * 0.42)),
            origin=Origin(
                xyz=(
                    tip_anchor + pad_length * 0.5 - 0.001,
                    0.0,
                    -body_height * 0.24,
                )
            ),
            material=pad_material,
            name="tip_pad",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="anthropomorphic_gripper_palm", assets=ASSETS)

    palm_frame = model.material("palm_frame", rgba=(0.27, 0.29, 0.32, 1.0))
    link_metal = model.material("link_metal", rgba=(0.66, 0.69, 0.73, 1.0))
    pin_dark = model.material("pin_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    pad_rubber = model.material("pad_rubber", rgba=(0.11, 0.12, 0.13, 1.0))

    palm = model.part("palm")

    palm_plate_profile = [
        (-0.060, -0.054),
        (-0.050, -0.070),
        (-0.010, -0.074),
        (0.020, -0.074),
        (0.046, -0.066),
        (0.060, -0.048),
        (0.060, 0.052),
        (0.050, 0.068),
        (0.030, 0.074),
        (-0.042, 0.074),
        (-0.056, 0.062),
        (-0.060, 0.046),
    ]
    palm_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(palm_plate_profile, PALM_THICKNESS, center=True),
        ASSETS.mesh_path("gripper_palm_plate.obj"),
    )
    palm.visual(
        palm_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, PALM_THICKNESS * 0.5)),
        material=palm_frame,
        name="palm_plate",
    )
    palm.visual(
        Box((0.020, PALM_WIDTH - 0.030, 0.010)),
        origin=Origin(xyz=(0.043, 0.0, 0.017)),
        material=palm_frame,
        name="front_rib",
    )
    palm.visual(
        Box((0.036, 0.050, 0.008)),
        origin=Origin(xyz=(-0.020, 0.0, 0.016)),
        material=palm_frame,
        name="back_boss",
    )

    finger_specs = [
        {
            "name": "index",
            "y": -0.039,
            "width": 0.016,
            "root_radius": 0.0062,
            "middle_radius": 0.0054,
            "distal_radius": 0.0047,
            "prox_span": 0.034,
            "mid_span": 0.026,
            "tip_extension": 0.021,
        },
        {
            "name": "middle",
            "y": -0.013,
            "width": 0.018,
            "root_radius": 0.0065,
            "middle_radius": 0.0056,
            "distal_radius": 0.0049,
            "prox_span": 0.036,
            "mid_span": 0.028,
            "tip_extension": 0.022,
        },
        {
            "name": "ring",
            "y": 0.013,
            "width": 0.018,
            "root_radius": 0.0064,
            "middle_radius": 0.0055,
            "distal_radius": 0.0048,
            "prox_span": 0.034,
            "mid_span": 0.026,
            "tip_extension": 0.021,
        },
        {
            "name": "little",
            "y": 0.039,
            "width": 0.015,
            "root_radius": 0.0058,
            "middle_radius": 0.0050,
            "distal_radius": 0.0044,
            "prox_span": 0.029,
            "mid_span": 0.022,
            "tip_extension": 0.018,
        },
    ]

    for spec in finger_specs:
        palm.visual(
            Box((0.016, spec["width"] + 0.006, 0.014)),
            origin=Origin(xyz=(PALM_FRONT_X - 0.013, spec["y"], 0.019)),
            material=palm_frame,
            name=f"{spec['name']}_mount",
        )

    palm.visual(
        Box((0.016, 0.034, 0.016)),
        origin=Origin(xyz=(0.000, -0.078, 0.018)),
        material=palm_frame,
        name="thumb_mount",
    )
    palm.visual(
        Box((0.018, 0.012, 0.010)),
        origin=Origin(xyz=(0.012, -0.054, 0.017)),
        material=palm_frame,
        name="thumb_brace",
    )
    palm.inertial = Inertial.from_geometry(
        Box((PALM_LENGTH, PALM_WIDTH, 0.030)),
        mass=0.85,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    finger_root_x_offset = PALM_FRONT_X + 0.0016
    finger_joint_z = 0.019
    digit_mass = {
        "proximal": 0.075,
        "middle": 0.055,
        "distal": 0.035,
    }

    for spec in finger_specs:
        proximal = model.part(f"{spec['name']}_proximal")
        _add_link_geometry(
            proximal,
            mesh_name=f"{spec['name']}_proximal_link_body",
            root_radius=spec["root_radius"],
            span_to_next_joint=spec["prox_span"],
            tip_extension=spec["prox_span"],
            body_width=spec["width"],
            body_height=0.0095,
            main_material=link_metal,
            pin_material=pin_dark,
            pad_material=pad_rubber,
            distal_radius=spec["middle_radius"],
        )
        proximal.inertial = Inertial.from_geometry(
            Box((spec["prox_span"] + spec["root_radius"], spec["width"], 0.014)),
            mass=digit_mass["proximal"],
            origin=Origin(xyz=(spec["prox_span"] * 0.45, 0.0, 0.0)),
        )

        middle = model.part(f"{spec['name']}_middle")
        _add_link_geometry(
            middle,
            mesh_name=f"{spec['name']}_middle_link_body",
            root_radius=spec["middle_radius"],
            span_to_next_joint=spec["mid_span"],
            tip_extension=spec["mid_span"],
            body_width=spec["width"] * 0.92,
            body_height=0.0088,
            main_material=link_metal,
            pin_material=pin_dark,
            pad_material=pad_rubber,
            distal_radius=spec["distal_radius"],
        )
        middle.inertial = Inertial.from_geometry(
            Box((spec["mid_span"] + spec["middle_radius"], spec["width"] * 0.92, 0.013)),
            mass=digit_mass["middle"],
            origin=Origin(xyz=(spec["mid_span"] * 0.42, 0.0, 0.0)),
        )

        distal = model.part(f"{spec['name']}_distal")
        _add_link_geometry(
            distal,
            mesh_name=f"{spec['name']}_distal_link_body",
            root_radius=spec["distal_radius"],
            span_to_next_joint=None,
            tip_extension=spec["tip_extension"],
            body_width=spec["width"] * 0.84,
            body_height=0.0078,
            main_material=link_metal,
            pin_material=pin_dark,
            pad_material=pad_rubber,
            add_tip_pad=True,
        )
        distal.inertial = Inertial.from_geometry(
            Box((spec["tip_extension"] + spec["distal_radius"], spec["width"] * 0.84, 0.011)),
            mass=digit_mass["distal"],
            origin=Origin(xyz=(spec["tip_extension"] * 0.48, 0.0, -0.001)),
        )

        model.articulation(
            f"{spec['name']}_root_joint",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(xyz=(finger_root_x_offset, spec["y"], finger_joint_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=2.5,
                lower=0.0,
                upper=FLEX_LIMIT,
            ),
        )
        model.articulation(
            f"{spec['name']}_middle_joint",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(spec["prox_span"], 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.2,
                velocity=3.0,
                lower=0.0,
                upper=FLEX_LIMIT,
            ),
        )
        model.articulation(
            f"{spec['name']}_distal_joint",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(spec["mid_span"], 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.6,
                velocity=3.0,
                lower=0.0,
                upper=FLEX_LIMIT,
            ),
        )

    thumb = {
        "root_radius": 0.0060,
        "distal_radius": 0.0052,
        "span": 0.029,
        "tip_extension": 0.023,
        "width": 0.020,
        "root_origin": (0.014, -0.080, 0.0175),
    }

    thumb_proximal = model.part("thumb_proximal")
    _add_link_geometry(
        thumb_proximal,
        mesh_name="thumb_proximal_link_body",
        root_radius=thumb["root_radius"],
        span_to_next_joint=thumb["span"],
        tip_extension=thumb["span"],
        body_width=thumb["width"],
        body_height=0.010,
        main_material=link_metal,
        pin_material=pin_dark,
        pad_material=pad_rubber,
        distal_radius=thumb["distal_radius"],
    )
    thumb_proximal.inertial = Inertial.from_geometry(
        Box((thumb["span"] + thumb["root_radius"], thumb["width"], 0.015)),
        mass=0.070,
        origin=Origin(xyz=(thumb["span"] * 0.45, 0.0, 0.0)),
    )

    thumb_distal = model.part("thumb_distal")
    _add_link_geometry(
        thumb_distal,
        mesh_name="thumb_distal_link_body",
        root_radius=thumb["distal_radius"],
        span_to_next_joint=None,
        tip_extension=thumb["tip_extension"],
        body_width=thumb["width"] * 0.88,
        body_height=0.0085,
        main_material=link_metal,
        pin_material=pin_dark,
        pad_material=pad_rubber,
        add_tip_pad=True,
    )
    thumb_distal.inertial = Inertial.from_geometry(
        Box((thumb["tip_extension"] + thumb["distal_radius"], thumb["width"] * 0.88, 0.012)),
        mass=0.040,
        origin=Origin(xyz=(thumb["tip_extension"] * 0.48, 0.0, -0.001)),
    )

    model.articulation(
        "thumb_root_joint",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=thumb_proximal,
        origin=Origin(xyz=thumb["root_origin"]),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=2.5,
            lower=0.0,
            upper=FLEX_LIMIT,
        ),
    )
    model.articulation(
        "thumb_distal_joint",
        ArticulationType.REVOLUTE,
        parent=thumb_proximal,
        child=thumb_distal,
        origin=Origin(xyz=(thumb["span"], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=3.0,
            lower=0.0,
            upper=FLEX_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    palm = object_model.get_part("palm")
    index_proximal = object_model.get_part("index_proximal")
    index_middle = object_model.get_part("index_middle")
    index_distal = object_model.get_part("index_distal")
    middle_proximal = object_model.get_part("middle_proximal")
    middle_middle = object_model.get_part("middle_middle")
    middle_distal = object_model.get_part("middle_distal")
    ring_proximal = object_model.get_part("ring_proximal")
    ring_middle = object_model.get_part("ring_middle")
    ring_distal = object_model.get_part("ring_distal")
    little_proximal = object_model.get_part("little_proximal")
    little_middle = object_model.get_part("little_middle")
    little_distal = object_model.get_part("little_distal")
    thumb_proximal = object_model.get_part("thumb_proximal")
    thumb_distal = object_model.get_part("thumb_distal")

    index_root_joint = object_model.get_articulation("index_root_joint")
    index_middle_joint = object_model.get_articulation("index_middle_joint")
    index_distal_joint = object_model.get_articulation("index_distal_joint")
    middle_root_joint = object_model.get_articulation("middle_root_joint")
    middle_middle_joint = object_model.get_articulation("middle_middle_joint")
    middle_distal_joint = object_model.get_articulation("middle_distal_joint")
    ring_root_joint = object_model.get_articulation("ring_root_joint")
    ring_middle_joint = object_model.get_articulation("ring_middle_joint")
    ring_distal_joint = object_model.get_articulation("ring_distal_joint")
    little_root_joint = object_model.get_articulation("little_root_joint")
    little_middle_joint = object_model.get_articulation("little_middle_joint")
    little_distal_joint = object_model.get_articulation("little_distal_joint")
    thumb_root_joint = object_model.get_articulation("thumb_root_joint")
    thumb_distal_joint = object_model.get_articulation("thumb_distal_joint")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    def elem_center(part, visual_name: str) -> tuple[float, float, float]:
        bounds = ctx.part_element_world_aabb(part, elem=part.get_visual(visual_name))
        assert bounds is not None
        low, high = bounds
        return (
            (low[0] + high[0]) * 0.5,
            (low[1] + high[1]) * 0.5,
            (low[2] + high[2]) * 0.5,
        )

    finger_layout = [
        ("index", index_proximal),
        ("middle", middle_proximal),
        ("ring", ring_proximal),
        ("little", little_proximal),
    ]
    for name, proximal in finger_layout:
        ctx.expect_gap(
            proximal,
            palm,
            axis="x",
            max_gap=0.0008,
            max_penetration=0.0,
            positive_elem=proximal.get_visual("root_barrel"),
            negative_elem=palm.get_visual(f"{name}_mount"),
            name=f"{name}_root_mount_gap",
        )
        ctx.expect_overlap(
            proximal,
            palm,
            axes="yz",
            min_overlap=0.008,
            elem_a=proximal.get_visual("root_barrel"),
            elem_b=palm.get_visual(f"{name}_mount"),
            name=f"{name}_root_mount_overlap",
        )

    link_pairs = [
        ("index_mid_seat", index_middle, index_proximal),
        ("index_tip_seat", index_distal, index_middle),
        ("middle_mid_seat", middle_middle, middle_proximal),
        ("middle_tip_seat", middle_distal, middle_middle),
        ("ring_mid_seat", ring_middle, ring_proximal),
        ("ring_tip_seat", ring_distal, ring_middle),
        ("little_mid_seat", little_middle, little_proximal),
        ("little_tip_seat", little_distal, little_middle),
        ("thumb_tip_seat", thumb_distal, thumb_proximal),
    ]
    for check_name, child, parent in link_pairs:
        ctx.expect_gap(
            child,
            parent,
            axis="x",
            max_gap=0.0008,
            max_penetration=1e-6,
            positive_elem=child.get_visual("root_barrel"),
            negative_elem=parent.get_visual("link_body"),
            name=f"{check_name}_gap",
        )
        ctx.expect_overlap(
            child,
            parent,
            axes="yz",
            min_overlap=0.006,
            elem_a=child.get_visual("root_barrel"),
            elem_b=parent.get_visual("link_body"),
            name=f"{check_name}_overlap",
        )

    ctx.expect_gap(
        thumb_proximal,
        palm,
        axis="x",
        max_gap=0.0008,
        max_penetration=0.0,
        positive_elem=thumb_proximal.get_visual("root_barrel"),
        negative_elem=palm.get_visual("thumb_mount"),
        name="thumb_root_mount_gap",
    )
    ctx.expect_overlap(
        thumb_proximal,
        palm,
        axes="yz",
        min_overlap=0.008,
        elem_a=thumb_proximal.get_visual("root_barrel"),
        elem_b=palm.get_visual("thumb_mount"),
        name="thumb_root_mount_overlap",
    )

    ctx.expect_origin_gap(middle_proximal, index_proximal, axis="y", min_gap=0.020, max_gap=0.032)
    ctx.expect_origin_gap(ring_proximal, middle_proximal, axis="y", min_gap=0.020, max_gap=0.032)
    ctx.expect_origin_gap(little_proximal, ring_proximal, axis="y", min_gap=0.020, max_gap=0.032)
    ctx.expect_origin_gap(index_proximal, thumb_proximal, axis="x", min_gap=0.035, max_gap=0.060)
    ctx.expect_origin_gap(index_proximal, thumb_proximal, axis="y", min_gap=0.030, max_gap=0.050)

    motion_checks = [
        ("index_root", index_root_joint, index_distal),
        ("index_middle", index_middle_joint, index_distal),
        ("index_distal", index_distal_joint, index_distal),
        ("middle_root", middle_root_joint, middle_distal),
        ("middle_middle", middle_middle_joint, middle_distal),
        ("middle_distal", middle_distal_joint, middle_distal),
        ("ring_root", ring_root_joint, ring_distal),
        ("ring_middle", ring_middle_joint, ring_distal),
        ("ring_distal", ring_distal_joint, ring_distal),
        ("little_root", little_root_joint, little_distal),
        ("little_middle", little_middle_joint, little_distal),
        ("little_distal", little_distal_joint, little_distal),
        ("thumb_root", thumb_root_joint, thumb_distal),
        ("thumb_distal", thumb_distal_joint, thumb_distal),
    ]
    for name, articulation, distal_part in motion_checks:
        rest_center = elem_center(distal_part, "tip_pad")
        with ctx.pose({articulation: math.radians(70.0)}):
            flex_center = elem_center(distal_part, "tip_pad")
            ctx.check(
                f"{name}_flexes_in_xz_plane",
                flex_center[2] < rest_center[2] - 0.004
                and abs(flex_center[1] - rest_center[1]) < 0.0015,
                details=(
                    f"rest={rest_center}, flex={flex_center}; "
                    "expected distal pad to move downward with negligible lateral drift."
                ),
            )

    index_tip_rest = elem_center(index_distal, "tip_pad")
    little_tip_rest = elem_center(little_distal, "tip_pad")
    thumb_tip_rest = elem_center(thumb_distal, "tip_pad")
    with ctx.pose({index_root_joint: math.radians(70.0)}):
        index_tip_flex = elem_center(index_distal, "tip_pad")
        little_tip_same = elem_center(little_distal, "tip_pad")
        thumb_tip_same = elem_center(thumb_distal, "tip_pad")
        ctx.check(
            "digits_are_uncoupled",
            index_tip_flex[2] < index_tip_rest[2] - 0.015
            and abs(little_tip_same[2] - little_tip_rest[2]) < 1e-6
            and abs(thumb_tip_same[2] - thumb_tip_rest[2]) < 1e-6,
            details=(
                f"index rest/flex={index_tip_rest}/{index_tip_flex}, "
                f"little rest/live={little_tip_rest}/{little_tip_same}, "
                f"thumb rest/live={thumb_tip_rest}/{thumb_tip_same}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
