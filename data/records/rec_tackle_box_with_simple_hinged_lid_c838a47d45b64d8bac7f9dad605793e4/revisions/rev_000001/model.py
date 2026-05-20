from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_tackle_box")

    safety_yellow = model.material("powder_coated_safety_yellow", rgba=(0.95, 0.68, 0.10, 1.0))
    dark_steel = model.material("dark_phosphate_steel", rgba=(0.10, 0.11, 0.11, 1.0))
    wear_plate = model.material("galvanized_wear_plate", rgba=(0.55, 0.58, 0.56, 1.0))
    red_lockout = model.material("red_lockout_plastic", rgba=(0.85, 0.03, 0.02, 1.0))
    black_rubber = model.material("black_rubber_bumper", rgba=(0.02, 0.02, 0.018, 1.0))

    # Root shell: an open, heavy-gauge tackle-box tray with top rims, hinge doubler
    # plates, latch keepers, bolted reinforcements, and skid feet.  The geometry is
    # intentionally plate-and-brace based rather than styled surfacing.
    base = model.part("base")
    base.visual(Box((0.72, 0.40, 0.035)), origin=Origin(xyz=(0.0, 0.0, 0.0175)), material=safety_yellow, name="floor")
    base.visual(Box((0.72, 0.035, 0.190)), origin=Origin(xyz=(0.0, -0.1825, 0.130)), material=safety_yellow, name="front_wall")
    base.visual(Box((0.72, 0.035, 0.190)), origin=Origin(xyz=(0.0, 0.1825, 0.130)), material=safety_yellow, name="back_wall")
    base.visual(Box((0.035, 0.330, 0.190)), origin=Origin(xyz=(-0.3425, 0.0, 0.130)), material=safety_yellow, name="end_wall_0")
    base.visual(Box((0.035, 0.330, 0.190)), origin=Origin(xyz=(0.3425, 0.0, 0.130)), material=safety_yellow, name="end_wall_1")

    base.visual(Box((0.76, 0.055, 0.025)), origin=Origin(xyz=(0.0, -0.1825, 0.2375)), material=dark_steel, name="front_rim")
    base.visual(Box((0.76, 0.055, 0.025)), origin=Origin(xyz=(0.0, 0.1825, 0.2375)), material=dark_steel, name="back_rim")
    base.visual(Box((0.055, 0.330, 0.025)), origin=Origin(xyz=(-0.3425, 0.0, 0.2375)), material=dark_steel, name="end_rim_0")
    base.visual(Box((0.055, 0.330, 0.025)), origin=Origin(xyz=(0.3425, 0.0, 0.2375)), material=dark_steel, name="end_rim_1")

    # External reinforcement ribs and corner load paths.
    for i, x in enumerate((-0.30, -0.10, 0.10, 0.30)):
        base.visual(Box((0.028, 0.012, 0.160)), origin=Origin(xyz=(x, -0.206, 0.125)), material=wear_plate, name=f"front_rib_{i}")
        base.visual(Box((0.028, 0.012, 0.160)), origin=Origin(xyz=(x, 0.206, 0.125)), material=wear_plate, name=f"back_rib_{i}")
    for i, x in enumerate((-0.32, 0.32)):
        base.visual(Box((0.050, 0.050, 0.185)), origin=Origin(xyz=(x, -0.175, 0.1275)), material=dark_steel, name=f"front_corner_post_{i}")
        base.visual(Box((0.050, 0.050, 0.185)), origin=Origin(xyz=(x, 0.175, 0.1275)), material=dark_steel, name=f"back_corner_post_{i}")

    # Twin latch keepers tied directly into the front wall by backer plates.
    for i, x in enumerate((-0.18, 0.18)):
        base.visual(Box((0.105, 0.020, 0.070)), origin=Origin(xyz=(x, -0.210, 0.126)), material=wear_plate, name=f"keeper_backer_{i}")
        base.visual(Box((0.092, 0.030, 0.060)), origin=Origin(xyz=(x, -0.235, 0.124)), material=dark_steel, name=f"keeper_{i}")
        base.visual(Box((0.092, 0.035, 0.012)), origin=Origin(xyz=(x, -0.2375, 0.087)), material=dark_steel, name=f"keeper_lip_{i}")
        for j, (dx, z) in enumerate(((-0.034, 0.102), (0.034, 0.102), (-0.034, 0.154), (0.034, 0.154))):
            base.visual(Cylinder(radius=0.006, length=0.006), origin=Origin(xyz=(x + dx, -0.223, z), rpy=(pi / 2.0, 0.0, 0.0)), material=dark_steel, name=f"keeper_bolt_{i}_{j}")

    # Rear hinge hardware: alternating knuckles plus a continuous captured pin.
    base.visual(Cylinder(radius=0.006, length=0.74), origin=Origin(xyz=(0.0, 0.215, 0.315), rpy=(0.0, pi / 2.0, 0.0)), material=dark_steel, name="main_hinge_pin")
    for i, x in enumerate((-0.255, 0.255)):
        base.visual(Cylinder(radius=0.018, length=0.125), origin=Origin(xyz=(x, 0.215, 0.315), rpy=(0.0, pi / 2.0, 0.0)), material=dark_steel, name=f"base_hinge_knuckle_{i}")
        base.visual(Box((0.135, 0.012, 0.090)), origin=Origin(xyz=(x, 0.206, 0.280)), material=wear_plate, name=f"base_hinge_leaf_{i}")
        for j, dx in enumerate((-0.040, 0.040)):
            base.visual(Cylinder(radius=0.006, length=0.006), origin=Origin(xyz=(x + dx, 0.214, 0.280), rpy=(-pi / 2.0, 0.0, 0.0)), material=dark_steel, name=f"hinge_leaf_bolt_{i}_{j}")
    for i, x in enumerate((-0.255, 0.255)):
        base.visual(Box((0.075, 0.035, 0.095)), origin=Origin(xyz=(x, 0.245, 0.280)), material=black_rubber, name=f"overtravel_stop_{i}")

    # Skid feet are wide and short so the box reads heavy and stable.
    for i, x in enumerate((-0.24, 0.24)):
        base.visual(Box((0.170, 0.050, 0.018)), origin=Origin(xyz=(x, -0.120, -0.009)), material=black_rubber, name=f"front_skid_{i}")
        base.visual(Box((0.170, 0.050, 0.018)), origin=Origin(xyz=(x, 0.120, -0.009)), material=black_rubber, name=f"rear_skid_{i}")

    # Lid frame is at the rear hinge axis.  Closed lid extends along local -Y.
    lid = model.part("lid")
    lid.visual(Box((0.76, 0.405, 0.035)), origin=Origin(xyz=(0.0, -0.2275, -0.0475)), material=safety_yellow, name="lid_panel")
    lid.visual(Box((0.76, 0.025, 0.065)), origin=Origin(xyz=(0.0, -0.4425, -0.0975)), material=safety_yellow, name="front_skirt")
    lid.visual(Box((0.025, 0.405, 0.065)), origin=Origin(xyz=(-0.3925, -0.2275, -0.0975)), material=safety_yellow, name="side_skirt_0")
    lid.visual(Box((0.025, 0.405, 0.065)), origin=Origin(xyz=(0.3925, -0.2275, -0.0975)), material=safety_yellow, name="side_skirt_1")
    lid.visual(Box((0.66, 0.060, 0.012)), origin=Origin(xyz=(0.0, -0.045, -0.024)), material=wear_plate, name="rear_doubler")
    lid.visual(Box((0.48, 0.030, 0.010)), origin=Origin(xyz=(0.0, -0.431, -0.064)), material=wear_plate, name="front_doubler")

    # Lid hinge knuckles alternate with the base knuckles and ride on the base pin.
    for name, x, length in (("lid_hinge_knuckle_center", 0.0, 0.205), ("lid_hinge_knuckle_0", -0.355, 0.060), ("lid_hinge_knuckle_1", 0.355, 0.060)):
        lid.visual(Cylinder(radius=0.018, length=length), origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=dark_steel, name=name)
        lid.visual(Box((length, 0.045, 0.060)), origin=Origin(xyz=(x, -0.028, -0.030)), material=wear_plate, name=f"{name}_strap")

    # Guarded lockout rail and bolt-on latch clevises.  The red sliding bar is a
    # separate articulated child; rails here show the capture path.
    lid.visual(Box((0.50, 0.010, 0.010)), origin=Origin(xyz=(0.0, -0.461, -0.055)), material=dark_steel, name="lock_top_rail")
    lid.visual(Box((0.50, 0.010, 0.010)), origin=Origin(xyz=(0.0, -0.461, -0.097)), material=dark_steel, name="lock_bottom_rail")
    lid.visual(Box((0.025, 0.012, 0.048)), origin=Origin(xyz=(-0.255, -0.461, -0.073)), material=dark_steel, name="lock_end_stop_0")
    lid.visual(Box((0.025, 0.012, 0.048)), origin=Origin(xyz=(0.255, -0.461, -0.073)), material=dark_steel, name="lock_end_stop_1")

    for i, x in enumerate((-0.18, 0.18)):
        lid.visual(Cylinder(radius=0.0045, length=0.115), origin=Origin(xyz=(x, -0.466, -0.115), rpy=(0.0, pi / 2.0, 0.0)), material=dark_steel, name=f"latch_pin_{i}")
        lid.visual(Box((0.012, 0.020, 0.040)), origin=Origin(xyz=(x - 0.052, -0.458, -0.115)), material=dark_steel, name=f"latch_clevis_0_{i}")
        lid.visual(Box((0.012, 0.020, 0.040)), origin=Origin(xyz=(x + 0.052, -0.458, -0.115)), material=dark_steel, name=f"latch_clevis_1_{i}")
        lid.visual(Box((0.140, 0.014, 0.010)), origin=Origin(xyz=(x, -0.461, -0.146)), material=dark_steel, name=f"latch_guard_bridge_{i}")
        lid.visual(Box((0.014, 0.018, 0.100)), origin=Origin(xyz=(x - 0.068, -0.461, -0.153)), material=dark_steel, name=f"latch_guard_side_0_{i}")
        lid.visual(Box((0.014, 0.018, 0.100)), origin=Origin(xyz=(x + 0.068, -0.461, -0.153)), material=dark_steel, name=f"latch_guard_side_1_{i}")

    # Load-spreading diagonal braces on top of the lid, running from the latch
    # guard line back toward the hinge doubler.
    for i, x in enumerate((-0.18, 0.18)):
        lid.visual(Box((0.020, 0.230, 0.012)), origin=Origin(xyz=(x, -0.255, -0.027), rpy=(0.0, 0.0, 0.28 if x < 0.0 else -0.28)), material=wear_plate, name=f"lid_brace_{i}")
        for j, y in enumerate((-0.345, -0.180)):
            lid.visual(Cylinder(radius=0.006, length=0.005), origin=Origin(xyz=(x, y, -0.0275)), material=dark_steel, name=f"brace_bolt_{i}_{j}")

    # Sliding safety lockout bar, constrained by visible rails and end stops.
    lock_bar = model.part("lock_bar")
    lock_bar.visual(Box((0.420, 0.012, 0.024)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=red_lockout, name="bar")
    lock_bar.visual(Box((0.050, 0.016, 0.032)), origin=Origin(xyz=(-0.170, -0.002, 0.0)), material=red_lockout, name="thumb_pad_0")
    lock_bar.visual(Box((0.050, 0.016, 0.032)), origin=Origin(xyz=(0.170, -0.002, 0.0)), material=red_lockout, name="thumb_pad_1")

    # Two over-center latch flaps.  Each has its own hinge barrel riding on a
    # lid-mounted pin, a flat load path down to the keeper, and a hook lip under
    # the keeper shelf.
    for i, x in enumerate((-0.18, 0.18)):
        latch = model.part(f"latch_{i}")
        latch.visual(Cylinder(radius=0.010, length=0.075), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=dark_steel, name="hinge_barrel")
        latch.visual(Box((0.078, 0.012, 0.120)), origin=Origin(xyz=(0.0, -0.006, -0.060)), material=dark_steel, name="plate")
        latch.visual(Box((0.078, 0.030, 0.012)), origin=Origin(xyz=(0.0, 0.004, -0.122)), material=dark_steel, name="hook_lip")
        latch.visual(Box((0.055, 0.014, 0.040)), origin=Origin(xyz=(0.0, -0.013, -0.036)), material=wear_plate, name="pull_tab")

        model.articulation(
            f"lid_to_latch_{i}",
            ArticulationType.REVOLUTE,
            parent=lid,
            child=latch,
            origin=Origin(xyz=(x, -0.466, -0.115)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=0.0, upper=1.15),
            motion_properties=MotionProperties(damping=0.08, friction=0.15),
        )

    model.articulation(
        "base_to_lid",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lid,
        origin=Origin(xyz=(0.0, 0.215, 0.315)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.2, lower=0.0, upper=1.12),
        motion_properties=MotionProperties(damping=0.20, friction=0.25),
    )

    model.articulation(
        "lid_to_lock_bar",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=lock_bar,
        origin=Origin(xyz=(0.0, -0.461, -0.075)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.18, lower=0.0, upper=0.055),
        motion_properties=MotionProperties(damping=0.04, friction=0.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lid = object_model.get_part("lid")
    lock_bar = object_model.get_part("lock_bar")
    latch_0 = object_model.get_part("latch_0")
    latch_1 = object_model.get_part("latch_1")
    lid_hinge = object_model.get_articulation("base_to_lid")
    lock_slide = object_model.get_articulation("lid_to_lock_bar")
    latch_joint_0 = object_model.get_articulation("lid_to_latch_0")
    latch_joint_1 = object_model.get_articulation("lid_to_latch_1")

    # Captured pins intentionally pass through hinge/latch barrels.  These are
    # local, hidden overlaps representing shafts inside bushings rather than
    # broad part collisions.
    for elem in ("lid_hinge_knuckle_center", "lid_hinge_knuckle_0", "lid_hinge_knuckle_1"):
        ctx.allow_overlap(
            base,
            lid,
            elem_a="main_hinge_pin",
            elem_b=elem,
            reason="The continuous hinge pin is intentionally captured inside the lid hinge knuckle.",
        )
        ctx.expect_within(base, lid, axes="yz", inner_elem="main_hinge_pin", outer_elem=elem, margin=0.001, name=f"{elem} surrounds hinge pin")
        ctx.expect_overlap(base, lid, axes="x", elem_a="main_hinge_pin", elem_b=elem, min_overlap=0.045, name=f"{elem} has pin engagement")

    for latch, elem, joint in ((latch_0, "latch_pin_0", latch_joint_0), (latch_1, "latch_pin_1", latch_joint_1)):
        ctx.allow_overlap(
            lid,
            latch,
            elem_a=elem,
            elem_b="hinge_barrel",
            reason="The latch pivot pin is intentionally captured inside the latch hinge barrel.",
        )
        ctx.expect_within(lid, latch, axes="yz", inner_elem=elem, outer_elem="hinge_barrel", margin=0.001, name=f"{latch.name} barrel surrounds pivot pin")
        ctx.expect_overlap(lid, latch, axes="x", elem_a=elem, elem_b="hinge_barrel", min_overlap=0.060, name=f"{latch.name} pivot has pin engagement")

    with ctx.pose({lid_hinge: 0.0, latch_joint_0: 0.0, latch_joint_1: 0.0, lock_slide: 0.0}):
        ctx.expect_gap(lid, base, axis="z", positive_elem="lid_panel", negative_elem="front_rim", max_gap=0.001, max_penetration=0.0, name="closed lid panel seats on front rim")
        ctx.expect_overlap(lid, base, axes="xy", elem_a="lid_panel", elem_b="front_rim", min_overlap=0.050, name="lid covers front rim")
        for latch, keeper in ((latch_0, "keeper_0"), (latch_1, "keeper_1")):
            ctx.expect_gap(base, latch, axis="y", positive_elem=keeper, negative_elem="plate", min_gap=0.0, max_gap=0.006, name=f"{latch.name} sits just outside keeper")
            ctx.expect_overlap(base, latch, axes="xz", elem_a=keeper, elem_b="plate", min_overlap=0.045, name=f"{latch.name} aligns with keeper")
            ctx.expect_gap(base, latch, axis="z", positive_elem=keeper, negative_elem="hook_lip", min_gap=0.0, max_gap=0.010, name=f"{latch.name} hook is constrained below keeper")

    rest_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
    with ctx.pose({lid_hinge: 1.12}):
        raised_front = ctx.part_element_world_aabb(lid, elem="front_skirt")
    ctx.check(
        "lid opens upward on rear hinge",
        rest_front is not None and raised_front is not None and raised_front[0][2] > rest_front[1][2] + 0.18,
        details=f"closed={rest_front}, open={raised_front}",
    )

    rest_lock = ctx.part_world_position(lock_bar)
    with ctx.pose({lock_slide: 0.055}):
        shifted_lock = ctx.part_world_position(lock_bar)
    ctx.check(
        "lockout bar slides laterally in guarded rail",
        rest_lock is not None and shifted_lock is not None and shifted_lock[0] > rest_lock[0] + 0.045,
        details=f"rest={rest_lock}, shifted={shifted_lock}",
    )

    for latch, joint in ((latch_0, latch_joint_0), (latch_1, latch_joint_1)):
        closed_box = ctx.part_element_world_aabb(latch, elem="plate")
        with ctx.pose({joint: 1.15}):
            released_box = ctx.part_element_world_aabb(latch, elem="plate")
        ctx.check(
            f"{latch.name} rotates outward and upward for release",
            closed_box is not None
            and released_box is not None
            and released_box[0][1] < closed_box[0][1] - 0.025
            and released_box[0][2] > closed_box[0][2] + 0.020,
            details=f"closed={closed_box}, released={released_box}",
        )

    return ctx.report()


object_model = build_object_model()
