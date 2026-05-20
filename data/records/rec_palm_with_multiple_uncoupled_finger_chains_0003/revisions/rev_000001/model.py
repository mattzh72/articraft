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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

PALM_LENGTH = 0.075
PALM_WIDTH = 0.094
PALM_THICKNESS = 0.022
PALM_FRONT_X = PALM_LENGTH * 0.5
PALM_HALF_WIDTH = PALM_WIDTH * 0.5

FINGER_BASE_PAD_SIZE_X = 0.010
FINGER_BASE_PAD_OUTER_EXT = 0.006
THUMB_BASE_PAD_SIZE_Y = 0.011
THUMB_BASE_PAD_OUTER_EXT = 0.007

FLEXION_LIMIT = math.pi / 2.0
HINGE_CONTACT_TOL = 2.5e-5
FINGER_FLEX_POSE = (
    math.radians(70.0),
    math.radians(55.0),
    math.radians(35.0),
)
THUMB_FLEX_POSE = (
    math.radians(55.0),
    math.radians(40.0),
)

FINGER_SPECS = (
    {
        "name": "index",
        "mount_y": -0.026,
        "mount_z": 0.015,
        "lengths": (0.032, 0.022, 0.017),
        "widths": (0.0125, 0.0115, 0.0105),
        "thicknesses": (0.0110, 0.0100, 0.0090),
    },
    {
        "name": "middle",
        "mount_y": -0.009,
        "mount_z": 0.0155,
        "lengths": (0.036, 0.026, 0.019),
        "widths": (0.0135, 0.0125, 0.0115),
        "thicknesses": (0.0115, 0.0105, 0.0095),
    },
    {
        "name": "ring",
        "mount_y": 0.009,
        "mount_z": 0.015,
        "lengths": (0.034, 0.024, 0.018),
        "widths": (0.0130, 0.0120, 0.0110),
        "thicknesses": (0.0110, 0.0100, 0.0090),
    },
    {
        "name": "pinky",
        "mount_y": 0.026,
        "mount_z": 0.0145,
        "lengths": (0.028, 0.020, 0.015),
        "widths": (0.0115, 0.0105, 0.0095),
        "thicknesses": (0.0100, 0.0090, 0.0085),
    },
)

THUMB_SPEC = {
    "name": "thumb",
    "mount_x": 0.009,
    "mount_z": 0.010,
    "lengths": (0.028, 0.021),
    "widths": (0.0150, 0.0135),
    "thicknesses": (0.0125, 0.0110),
}


def _hinge_radius(thickness: float) -> float:
    return thickness * 0.5 + 0.0008


def _link_mass(length: float, width: float, thickness: float, hinge_radius: float) -> float:
    knuckle_length = width * 0.9
    volume = (length * width * thickness) + (math.pi * hinge_radius * hinge_radius * knuckle_length)
    return 1450.0 * volume


def _build_link_part(
    part,
    *,
    length: float,
    width: float,
    thickness: float,
    body_material,
    hinge_material,
    tip_material,
) -> float:
    hinge_radius = _hinge_radius(thickness)
    knuckle_length = width * 0.9

    part.visual(
        Cylinder(radius=hinge_radius, length=knuckle_length),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_material,
        name="knuckle",
    )
    part.visual(
        Box((length, width, thickness)),
        origin=Origin(xyz=(length * 0.5, 0.0, 0.0)),
        material=body_material,
        name="body",
    )
    part.visual(
        Box((length * 0.18, width * 0.82, thickness * 0.82)),
        origin=Origin(xyz=(length * 0.89, 0.0, 0.0)),
        material=tip_material,
        name="tip",
    )
    part.inertial = Inertial.from_geometry(
        Box((length + hinge_radius, width, max(thickness, hinge_radius * 2.0))),
        mass=_link_mass(length, width, thickness, hinge_radius),
        origin=Origin(xyz=((length - hinge_radius) * 0.5, 0.0, 0.0)),
    )
    return hinge_radius


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_hand_palm", assets=ASSETS)

    palm_material = model.material("palm_polymer", rgba=(0.18, 0.20, 0.23, 1.0))
    palm_mount_material = model.material("mount_block", rgba=(0.26, 0.29, 0.33, 1.0))
    phalanx_material = model.material("phalanx_shell", rgba=(0.77, 0.80, 0.83, 1.0))
    knuckle_material = model.material("hinge_metal", rgba=(0.42, 0.45, 0.49, 1.0))
    fingertip_material = model.material("fingertip_pad", rgba=(0.63, 0.67, 0.71, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((PALM_LENGTH, PALM_WIDTH, PALM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, PALM_THICKNESS * 0.5)),
        material=palm_material,
        name="palm_block",
    )
    palm.visual(
        Box((0.022, 0.040, 0.012)),
        origin=Origin(xyz=(-0.022, 0.0, 0.010)),
        material=palm_mount_material,
        name="wrist_stub",
    )

    for spec in FINGER_SPECS:
        palm.visual(
            Box((FINGER_BASE_PAD_SIZE_X, spec["widths"][0] + 0.003, spec["thicknesses"][0] + 0.004)),
            origin=Origin(
                xyz=(
                    PALM_FRONT_X + (FINGER_BASE_PAD_OUTER_EXT - (FINGER_BASE_PAD_SIZE_X - FINGER_BASE_PAD_OUTER_EXT)) * 0.5,
                    spec["mount_y"],
                    spec["mount_z"],
                )
            ),
            material=palm_mount_material,
            name=f"{spec['name']}_base_pad",
        )

    palm.visual(
        Box((0.018, THUMB_BASE_PAD_SIZE_Y, THUMB_SPEC["thicknesses"][0] + 0.004)),
        origin=Origin(
            xyz=(
                THUMB_SPEC["mount_x"],
                -(PALM_HALF_WIDTH + (THUMB_BASE_PAD_OUTER_EXT - (THUMB_BASE_PAD_SIZE_Y - THUMB_BASE_PAD_OUTER_EXT)) * 0.5),
                THUMB_SPEC["mount_z"],
            )
        ),
        material=palm_mount_material,
        name="thumb_base_pad",
    )
    palm.inertial = Inertial.from_geometry(
        Box((PALM_LENGTH + 0.018, PALM_WIDTH, PALM_THICKNESS)),
        mass=0.26,
        origin=Origin(xyz=(0.0, 0.0, PALM_THICKNESS * 0.5)),
    )

    for spec in FINGER_SPECS:
        segment_parts = []
        segment_radii = []
        for segment_name, length, width, thickness in zip(
            ("proximal", "middle", "distal"),
            spec["lengths"],
            spec["widths"],
            spec["thicknesses"],
        ):
            part = model.part(f"{spec['name']}_{segment_name}")
            segment_parts.append(part)
            segment_radii.append(
                _build_link_part(
                    part,
                    length=length,
                    width=width,
                    thickness=thickness,
                    body_material=phalanx_material,
                    hinge_material=knuckle_material,
                    tip_material=fingertip_material,
                )
            )

        model.articulation(
            f"palm_to_{spec['name']}_proximal",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=segment_parts[0],
            origin=Origin(
                xyz=(
                    PALM_FRONT_X + FINGER_BASE_PAD_OUTER_EXT + segment_radii[0],
                    spec["mount_y"],
                    spec["mount_z"],
                )
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=3.0,
                velocity=3.0,
                lower=0.0,
                upper=FLEXION_LIMIT,
            ),
        )
        model.articulation(
            f"{spec['name']}_proximal_to_middle",
            ArticulationType.REVOLUTE,
            parent=segment_parts[0],
            child=segment_parts[1],
            origin=Origin(xyz=(spec["lengths"][0] + segment_radii[1], 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.5,
                velocity=3.0,
                lower=0.0,
                upper=FLEXION_LIMIT,
            ),
        )
        model.articulation(
            f"{spec['name']}_middle_to_distal",
            ArticulationType.REVOLUTE,
            parent=segment_parts[1],
            child=segment_parts[2],
            origin=Origin(xyz=(spec["lengths"][1] + segment_radii[2], 0.0, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=3.0,
                lower=0.0,
                upper=FLEXION_LIMIT,
            ),
        )

    thumb_parts = []
    thumb_radii = []
    for segment_name, length, width, thickness in zip(
        ("proximal", "distal"),
        THUMB_SPEC["lengths"],
        THUMB_SPEC["widths"],
        THUMB_SPEC["thicknesses"],
    ):
        part = model.part(f"thumb_{segment_name}")
        thumb_parts.append(part)
        thumb_radii.append(
            _build_link_part(
                part,
                length=length,
                width=width,
                thickness=thickness,
                body_material=phalanx_material,
                hinge_material=knuckle_material,
                tip_material=fingertip_material,
            )
        )

    model.articulation(
        "palm_to_thumb_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=thumb_parts[0],
        origin=Origin(
            xyz=(
                THUMB_SPEC["mount_x"],
                -(PALM_HALF_WIDTH + THUMB_BASE_PAD_OUTER_EXT + thumb_radii[0]),
                THUMB_SPEC["mount_z"],
            ),
            rpy=(0.0, 0.0, -math.pi / 2.0),
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.5,
            velocity=3.0,
            lower=0.0,
            upper=FLEXION_LIMIT,
        ),
    )
    model.articulation(
        "thumb_proximal_to_distal",
        ArticulationType.REVOLUTE,
        parent=thumb_parts[0],
        child=thumb_parts[1],
        origin=Origin(xyz=(THUMB_SPEC["lengths"][0] + thumb_radii[1], 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=FLEXION_LIMIT,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    palm = object_model.get_part("palm")

    finger_parts = {}
    finger_joints = {}
    finger_visuals = {}
    for spec in FINGER_SPECS:
        name = spec["name"]
        proximal = object_model.get_part(f"{name}_proximal")
        middle = object_model.get_part(f"{name}_middle")
        distal = object_model.get_part(f"{name}_distal")
        finger_parts[name] = (proximal, middle, distal)
        finger_joints[name] = (
            object_model.get_articulation(f"palm_to_{name}_proximal"),
            object_model.get_articulation(f"{name}_proximal_to_middle"),
            object_model.get_articulation(f"{name}_middle_to_distal"),
        )
        finger_visuals[name] = {
            "base_pad": palm.get_visual(f"{name}_base_pad"),
            "proximal_body": proximal.get_visual("body"),
            "proximal_knuckle": proximal.get_visual("knuckle"),
            "middle_body": middle.get_visual("body"),
            "middle_knuckle": middle.get_visual("knuckle"),
            "distal_knuckle": distal.get_visual("knuckle"),
        }

    thumb_proximal = object_model.get_part("thumb_proximal")
    thumb_distal = object_model.get_part("thumb_distal")
    thumb_base_joint = object_model.get_articulation("palm_to_thumb_proximal")
    thumb_distal_joint = object_model.get_articulation("thumb_proximal_to_distal")
    thumb_visuals = {
        "base_pad": palm.get_visual("thumb_base_pad"),
        "proximal_body": thumb_proximal.get_visual("body"),
        "proximal_knuckle": thumb_proximal.get_visual("knuckle"),
        "distal_knuckle": thumb_distal.get_visual("knuckle"),
    }

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts(contact_tol=HINGE_CONTACT_TOL)
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

    ctx.check(
        "expected_part_count",
        len(object_model.parts) == 15,
        details=f"expected 15 parts, found {len(object_model.parts)}",
    )
    ctx.check(
        "expected_articulation_count",
        len(object_model.articulations) == 14,
        details=f"expected 14 articulations, found {len(object_model.articulations)}",
    )

    for spec in FINGER_SPECS:
        name = spec["name"]
        proximal, middle, distal = finger_parts[name]
        visuals = finger_visuals[name]

        ctx.expect_contact(
            proximal,
            palm,
            contact_tol=HINGE_CONTACT_TOL,
            elem_a=visuals["proximal_knuckle"],
            elem_b=visuals["base_pad"],
            name=f"{name}_base_contact",
        )
        ctx.expect_overlap(
            proximal,
            palm,
            axes="yz",
            min_overlap=spec["thicknesses"][0] * 0.7,
            elem_a=visuals["proximal_knuckle"],
            elem_b=visuals["base_pad"],
            name=f"{name}_base_alignment",
        )
        ctx.expect_contact(
            middle,
            proximal,
            contact_tol=HINGE_CONTACT_TOL,
            elem_a=visuals["middle_knuckle"],
            elem_b=visuals["proximal_body"],
            name=f"{name}_middle_joint_contact",
        )
        ctx.expect_overlap(
            middle,
            proximal,
            axes="yz",
            min_overlap=spec["thicknesses"][1] * 0.65,
            elem_a=visuals["middle_knuckle"],
            elem_b=visuals["proximal_body"],
            name=f"{name}_middle_joint_alignment",
        )
        ctx.expect_contact(
            distal,
            middle,
            contact_tol=HINGE_CONTACT_TOL,
            elem_a=visuals["distal_knuckle"],
            elem_b=visuals["middle_body"],
            name=f"{name}_distal_joint_contact",
        )
        ctx.expect_overlap(
            distal,
            middle,
            axes="yz",
            min_overlap=spec["thicknesses"][2] * 0.65,
            elem_a=visuals["distal_knuckle"],
            elem_b=visuals["middle_body"],
            name=f"{name}_distal_joint_alignment",
        )

    ctx.expect_contact(
        thumb_proximal,
        palm,
        contact_tol=HINGE_CONTACT_TOL,
        elem_a=thumb_visuals["proximal_knuckle"],
        elem_b=thumb_visuals["base_pad"],
        name="thumb_base_contact",
    )
    ctx.expect_overlap(
        thumb_proximal,
        palm,
        axes="xz",
        min_overlap=THUMB_SPEC["thicknesses"][0] * 0.65,
        elem_a=thumb_visuals["proximal_knuckle"],
        elem_b=thumb_visuals["base_pad"],
        name="thumb_base_alignment",
    )
    ctx.expect_contact(
        thumb_distal,
        thumb_proximal,
        contact_tol=HINGE_CONTACT_TOL,
        elem_a=thumb_visuals["distal_knuckle"],
        elem_b=thumb_visuals["proximal_body"],
        name="thumb_distal_joint_contact",
    )
    ctx.expect_overlap(
        thumb_distal,
        thumb_proximal,
        axes="xz",
        min_overlap=THUMB_SPEC["thicknesses"][1] * 0.65,
        elem_a=thumb_visuals["distal_knuckle"],
        elem_b=thumb_visuals["proximal_body"],
        name="thumb_distal_joint_alignment",
    )

    index_origin = ctx.part_world_position(finger_parts["index"][0])
    middle_origin = ctx.part_world_position(finger_parts["middle"][0])
    ring_origin = ctx.part_world_position(finger_parts["ring"][0])
    pinky_origin = ctx.part_world_position(finger_parts["pinky"][0])
    thumb_origin = ctx.part_world_position(thumb_proximal)
    if (
        index_origin is not None
        and middle_origin is not None
        and ring_origin is not None
        and pinky_origin is not None
        and thumb_origin is not None
    ):
        ctx.check(
            "finger_bases_are_ordered_across_palm",
            index_origin[1] < middle_origin[1] < ring_origin[1] < pinky_origin[1],
            details=(
                f"finger base y order was "
                f"{index_origin[1]:.4f}, {middle_origin[1]:.4f}, {ring_origin[1]:.4f}, {pinky_origin[1]:.4f}"
            ),
        )
        ctx.check(
            "thumb_mount_is_lateral",
            thumb_origin[1] < index_origin[1] - 0.02,
            details=f"thumb y={thumb_origin[1]:.4f}, index y={index_origin[1]:.4f}",
        )

    for spec in FINGER_SPECS:
        name = spec["name"]
        proximal, middle, distal = finger_parts[name]
        base_joint, middle_joint, distal_joint = finger_joints[name]
        visuals = finger_visuals[name]
        rest_aabb = ctx.part_world_aabb(distal)
        if rest_aabb is None:
            ctx.fail(f"{name}_rest_aabb_available", "distal link had no measurable rest-pose bounds")
            continue
        with ctx.pose(
            {
                base_joint: FINGER_FLEX_POSE[0],
                middle_joint: FINGER_FLEX_POSE[1],
                distal_joint: FINGER_FLEX_POSE[2],
            }
        ):
            flex_aabb = ctx.part_world_aabb(distal)
            if flex_aabb is None:
                ctx.fail(f"{name}_flex_aabb_available", "distal link had no measurable flexed bounds")
            else:
                ctx.check(
                    f"{name}_curls_downward",
                    flex_aabb[0][2] < rest_aabb[0][2] - 0.010 and flex_aabb[1][0] < rest_aabb[1][0] - 0.008,
                    details=(
                        f"rest aabb={rest_aabb}, flexed aabb={flex_aabb}"
                    ),
                )
            ctx.expect_contact(
                proximal,
                palm,
                contact_tol=HINGE_CONTACT_TOL,
                elem_a=visuals["proximal_knuckle"],
                elem_b=visuals["base_pad"],
                name=f"{name}_base_contact_flexed",
            )
            ctx.expect_contact(
                middle,
                proximal,
                contact_tol=HINGE_CONTACT_TOL,
                elem_a=visuals["middle_knuckle"],
                elem_b=visuals["proximal_body"],
                name=f"{name}_middle_contact_flexed",
            )
            ctx.expect_contact(
                distal,
                middle,
                contact_tol=HINGE_CONTACT_TOL,
                elem_a=visuals["distal_knuckle"],
                elem_b=visuals["middle_body"],
                name=f"{name}_distal_contact_flexed",
            )

    thumb_rest_aabb = ctx.part_world_aabb(thumb_distal)
    if thumb_rest_aabb is None:
        ctx.fail("thumb_rest_aabb_available", "thumb distal link had no measurable rest-pose bounds")
    else:
        with ctx.pose({thumb_base_joint: THUMB_FLEX_POSE[0], thumb_distal_joint: THUMB_FLEX_POSE[1]}):
            thumb_flex_aabb = ctx.part_world_aabb(thumb_distal)
            if thumb_flex_aabb is None:
                ctx.fail("thumb_flex_aabb_available", "thumb distal link had no measurable flexed bounds")
            else:
                ctx.check(
                    "thumb_curls_inward",
                    thumb_flex_aabb[0][2] < thumb_rest_aabb[0][2] - 0.008
                    and thumb_flex_aabb[0][1] > thumb_rest_aabb[0][1] + 0.006,
                    details=f"rest aabb={thumb_rest_aabb}, flexed aabb={thumb_flex_aabb}",
                )
            ctx.expect_contact(
                thumb_proximal,
                palm,
                contact_tol=HINGE_CONTACT_TOL,
                elem_a=thumb_visuals["proximal_knuckle"],
                elem_b=thumb_visuals["base_pad"],
                name="thumb_base_contact_flexed",
            )
            ctx.expect_contact(
                thumb_distal,
                thumb_proximal,
                contact_tol=HINGE_CONTACT_TOL,
                elem_a=thumb_visuals["distal_knuckle"],
                elem_b=thumb_visuals["proximal_body"],
                name="thumb_distal_contact_flexed",
            )

    ctx.fail_if_articulation_overlaps(max_pose_samples=12)
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=12, ignore_adjacent=True, ignore_fixed=True)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
